#include "image_grabber.hpp"
#include "exception.hpp"
#include "camera.hpp"
#include "stamped_image.hpp"
#include "affinity.hpp"
#include <iostream>
#include <QTime>
#include <QThread>
#include <QElapsedTimer>
#include <QFileInfo>
#include <opencv2/core/core.hpp>
#include "video_utils.hpp"

// TEMPOERARY
// ----------------------------------------
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// ----------------------------------------

namespace bias {

    unsigned int ImageGrabber::DEFAULT_NUM_STARTUP_SKIP = 2;
    unsigned int ImageGrabber::MIN_STARTUP_SKIP = 1;
    unsigned int ImageGrabber::MAX_ERROR_COUNT = 500;

    ImageGrabber::ImageGrabber(QObject *parent) : QObject(parent) 
    {
        initialize(0,NULL,NULL);
    }

    ImageGrabber::ImageGrabber (
            unsigned int cameraNumber,
            std::shared_ptr<Lockable<Camera>> cameraPtr,
            std::shared_ptr<LockableQueue<StampedImage>> newImageQueuePtr, 
            QObject *parent
            ) : QObject(parent)
    {
        initialize(cameraNumber, cameraPtr, newImageQueuePtr);
    }

    void ImageGrabber::initialize( 
            unsigned int cameraNumber,
            std::shared_ptr<Lockable<Camera>> cameraPtr,
            std::shared_ptr<LockableQueue<StampedImage>> newImageQueuePtr 
            ) 
    {
        capturing_ = false;
        stopped_ = true;
        cameraPtr_ = cameraPtr;
        newImageQueuePtr_ = newImageQueuePtr;
        numStartUpSkip_ = DEFAULT_NUM_STARTUP_SKIP;
        cameraNumber_ = cameraNumber;
        if ((cameraPtr_ != NULL) && (newImageQueuePtr_ != NULL))
        {
            ready_ = true;
        }
        else
        {
            ready_ = false;
        }
        errorCountEnabled_ = true;

        // read from video instead
        isVideo_ = false;
        vidFileName_ = QString("");
        startFrame_ = 0;
        playFps_ = 0.0;

    }

    void ImageGrabber::setIsVideo(bool v) {
        isVideo_ = v;
    }
    void ImageGrabber::setVideoFileName(QString captureVideoFileName) {
        vidFileName_ = captureVideoFileName;
    }
    void ImageGrabber::setStartFrame(int f) {
        startFrame_ = (f > 0) ? f : 0;
    }
    void ImageGrabber::setPlayFps(double fps) {
        playFps_ = (fps > 0.0) ? fps : 0.0;
    }

    void ImageGrabber::initializeVidBackend()
    {
        printf("Reading from video file %s\n", vidFileName_.toStdString().c_str());
        vidObj_ = new videoBackend(vidFileName_);
        int vid_numFrames = vidObj_->getNumFrames();
        printf("Video has %d frames\n", vid_numFrames);
        vidObj_->checkCapOpen();
	}

    void ImageGrabber::stop()
    {
        stopped_ = true;
    }


    void ImageGrabber::enableErrorCount()
    {
        errorCountEnabled_ = true;
    }
   
    void ImageGrabber::disableErrorCount()
    {
        errorCountEnabled_ = false;
    }

    void ImageGrabber::run()
    { 
        bool isFirst = true;
        bool done = false;
        bool error = false;
        bool errorEmitted = false;
        unsigned int errorId = 0;
        unsigned int errorCount = 0;
        unsigned long frameCount = 0;
        unsigned long startUpCount = 0;
        double dtEstimate = 0.0;
        unsigned int nDtUpdates = 0;

        StampedImage stampImg;

        TimeStamp timeStamp;
        TimeStamp timeStampInit; 

        double timeStampDbl = 0.0;
        double timeStampDblLast = 0.0;

        QString errorMsg("no message");

        if (!ready_) 
        { 
            return; 
        }

        // Set thread priority to "time critical" and assign cpu affinity
        QThread *thisThread = QThread::currentThread();
        thisThread -> setPriority(QThread::TimeCriticalPriority);
        ThreadAffinityService::assignThreadAffinity(true,cameraNumber_);

        // Start image capture
        if (isVideo_) {
            initializeVidBackend();
            if (startFrame_ > 0) {
                // seek the video and number frames so JSON frame ~ video frame index
                // (same -2 startup-skip convention as a full run). NB: seeking is only as
                // accurate as the codec allows (may snap to the nearest keyframe).
                vidObj_->setFrame(startFrame_);
                frameCount = (unsigned long)startFrame_;
            }
        }
        else {
            cameraPtr_->acquireLock();
            try
            {
                cameraPtr_->startCapture();
            }
            catch (RuntimeError& runtimeError)
            {
                error = true;
                errorId = runtimeError.id();
                errorMsg = QString::fromStdString(runtimeError.what());
            }
            catch (...)
            {
                std::cout << "Unexpected exception in startCapture, camera " << cameraNumber_ << std::endl;
                error = true;
                errorId = ERROR_CAPTURE_UNEXPECTED_EXCEPTION;
                errorMsg = QString("Unexpected exception in startCapture");
            }
            cameraPtr_->releaseLock();

            if (error)
            {
                emit startCaptureError(errorId, errorMsg);
                errorEmitted = true;
                return;
            }
        }


        acquireLock();
        stopped_ = false;
        releaseLock();

        //// TEMPORARY - for mouse grab detector testing
        //// ------------------------------------------------------------------------------

        //// Check for existence of movie file
        //QString grabTestMovieFileName("bias_test.avi");
        //cv::VideoCapture fileCapture;
        //unsigned int numFrames = 0;
        //int fourcc = 0;
        //bool haveGrabTestMovie = false;

        //if (QFileInfo(grabTestMovieFileName).exists())
        //{
        //    fileCapture.open(grabTestMovieFileName.toStdString());
        //    if ( fileCapture.isOpened() )
        //    {
        //        numFrames = (unsigned int)(fileCapture.get(CV_CAP_PROP_FRAME_COUNT));
        //        fourcc = int(fileCapture.get(CV_CAP_PROP_FOURCC));
        //        haveGrabTestMovie = true;
        //    }
        //}
        //// -------------------------------------------------------------------------------
        

        // Optional video playback throttle: pace the grab loop to playFps_ so the video plays
        // at a realistic rate (like a real camera) rather than flat out. 0 = no throttle.
        QElapsedTimer playTimer;
        double playNextMs = 0.0;
        if (isVideo_ && playFps_ > 0.0) { playTimer.start(); }

        // Heartbeat: periodically log this camera's frame count so a stalled/crashed
        // run can be correlated against the last time frames were actually arriving.
        QElapsedTimer heartbeatTimer;
        heartbeatTimer.start();
        const qint64 heartbeatIntervalMs = 30000;
        unsigned long heartbeatLastFrameCount = 0;

        // Grab images from camera until the done signal is given
        while (!done)
        {
            acquireLock();
            done = stopped_;
            releaseLock();

            // Errors are per-frame: a successful grab after a failed one must
            // resume pushing frames and reset errorCount below.
            error = false;

            if (heartbeatTimer.elapsed() >= heartbeatIntervalMs)
            {
                std::cout << "HEARTBEAT: camera " << cameraNumber_
                          << ", frameCount = " << frameCount
                          << ", framesSinceLast = " << (frameCount - heartbeatLastFrameCount)
                          << std::endl;
                heartbeatLastFrameCount = frameCount;
                heartbeatTimer.restart();
            }

            // pace video playback to playFps_ (sleep until this frame's scheduled time)
            if (isVideo_ && playFps_ > 0.0)
            {
                qint64 nowMs = playTimer.elapsed();
                if (playNextMs < (double)nowMs) { playNextMs = (double)nowMs; } // don't burst-catch-up after a stall
                qint64 waitMs = (qint64)(playNextMs - (double)nowMs);
                if (waitMs > 0) { QThread::msleep((unsigned long)waitMs); }
                playNextMs += 1000.0 / playFps_;
            }

            // Grab an image
            cameraPtr_->acquireLock();
            if (isVideo_) {
                try
                {
                    stampImg.image = vidObj_->grabImage();
                    if (stampImg.image.empty()) {
                        done = true;
                    }
                    timeStamp = vidObj_->getImageTimeStamp();
                }
                catch (RuntimeError& runtimeError)
				{
					std::cout << "Video frame grab error: id = ";
					std::cout << runtimeError.id() << ", what = ";
					std::cout << runtimeError.what() << std::endl;
					error = true;
				}
				catch (...)
				{
					std::cout << "Unexpected exception during video frame grab" << std::endl;
					error = true;
				}
            }
            else {
                try
                {
                    stampImg.image = cameraPtr_->grabImage();
                    timeStamp = cameraPtr_->getImageTimeStamp();
                }
                catch (RuntimeError& runtimeError)
                {
                    std::cout << "Frame grab error: id = ";
                    std::cout << runtimeError.id() << ", what = ";
                    std::cout << runtimeError.what() << std::endl;
                    error = true;
                }
                catch (...)
                {
                    std::cout << "Unexpected exception during frame grab, camera " << cameraNumber_ << std::endl;
                    error = true;
                }
            }
            cameraPtr_->releaseLock();

            // grabImage is nonblocking - returned frame is empty is a new frame is not available.
            if (stampImg.image.empty()) 
            { 
                QThread::yieldCurrentThread();
                continue; 
            }
            
            // Push image into new image queue
            if (!error) 
            {
                errorCount = 0;                  // Reset error count 
                timeStampDblLast = timeStampDbl; // Save last timestamp
                
                // Set initial time stamp for fps estimate
                if ((startUpCount == 0) && (numStartUpSkip_ > 0))
                {
                    timeStampInit = timeStamp;
                }
                timeStampDbl = convertTimeStampToDouble(timeStamp, timeStampInit);

                // Skip some number of frames on startup - recommened by Point Grey. 
                // During this time compute running avg to get estimate of frame interval
                if (startUpCount < numStartUpSkip_)
                {
                    double dt = timeStampDbl - timeStampDblLast;
                    if (startUpCount == MIN_STARTUP_SKIP)
                    {
                        dtEstimate = dt;
                        nDtUpdates = 1;

                    }
                    else if (startUpCount > MIN_STARTUP_SKIP)
                    {
                        nDtUpdates++;
                        double c0 = double(nDtUpdates-1)/double(nDtUpdates);
                        double c1 = double(1.0)/double(nDtUpdates);
                        dtEstimate = c0*dtEstimate + c1*dt;
                    }
                    startUpCount++;
                    continue;
                }

                //std::cout << "dt grabber: " << timeStampDbl - timeStampDblLast << std::endl;
                
                // Reset initial time stamp for image acquisition
                if ((isFirst) && (startUpCount >= numStartUpSkip_))
                {
                    timeStampInit = timeStamp;
                    timeStampDblLast = 0.0;
                    isFirst = false;
                    timeStampDbl = convertTimeStampToDouble(timeStamp, timeStampInit);
                    emit startTimer();
                }
                //
                
                //// TEMPORARY - for mouse grab detector testing
                //// --------------------------------------------------------------------- 
                //cv::Mat fileMat;
                //StampedImage fileImg;
                //if (haveGrabTestMovie)
                //{
                //    fileCapture >> fileMat; 
                //    if (fileMat.empty())
                //    {
                //        fileCapture.set(CV_CAP_PROP_POS_FRAMES,0);
                //        continue;
                //    }

                //    cv::Mat  fileMatMono = cv::Mat(fileMat.size(), CV_8UC1);
                //    cvtColor(fileMat, fileMatMono, CV_RGB2GRAY);
                //    
                //    cv::Mat camSizeImage = cv::Mat(stampImg.image.size(), CV_8UC1);
                //    int padx = camSizeImage.rows - fileMatMono.rows;
                //    int pady = camSizeImage.cols - fileMatMono.cols;

                //    cv::Scalar padColor = cv::Scalar(0);
                //    cv::copyMakeBorder(fileMatMono, camSizeImage, 0, pady, 0, padx, cv::BORDER_CONSTANT, cv::Scalar(0));
                //    stampImg.image = camSizeImage;
                //}
                //// ---------------------------------------------------------------------
                
                // Set image data timestamp, framecount and frame interval estimate
                stampImg.timeStamp = timeStampDbl;
                stampImg.frameCount = frameCount;
                stampImg.dtEstimate = dtEstimate;
                frameCount++;

                newImageQueuePtr_ -> acquireLock();
                newImageQueuePtr_ -> push(stampImg);
                newImageQueuePtr_ -> signalNotEmpty(); 
                newImageQueuePtr_ -> releaseLock();

            }
            else
            {
                if (errorCountEnabled_ ) 
                {
                    errorCount++;
                    if (errorCount > MAX_ERROR_COUNT)
                    {
                        errorId = ERROR_CAPTURE_MAX_ERROR_COUNT;
                        errorMsg = QString("Maximum allowed capture error count reached");
                        if (!errorEmitted) 
                        {
                            emit captureError(errorId, errorMsg);
                            errorEmitted = true;
                        }
                    }
                }
            }

        } // while (!done) 

        // Stop image capture
        error = false;
        if (isVideo_) {
            vidObj_->releaseCapObject();
        }
        else {
            cameraPtr_->acquireLock();
            try
            {
                cameraPtr_->stopCapture();
            }
            catch (RuntimeError& runtimeError)
            {
                error = true;
                errorId = runtimeError.id();
                errorMsg = QString::fromStdString(runtimeError.what());
            }
            catch (...)
            {
                std::cout << "Unexpected exception in stopCapture, camera " << cameraNumber_ << std::endl;
                error = true;
                errorId = ERROR_CAPTURE_UNEXPECTED_EXCEPTION;
                errorMsg = QString("Unexpected exception in stopCapture");
            }
            cameraPtr_->releaseLock();
        }
        if ((error) && (!errorEmitted))
        { 
            emit stopCaptureError(errorId, errorMsg);
        }

    }


    double ImageGrabber::convertTimeStampToDouble(TimeStamp curr, TimeStamp init)
    {
        double timeStampDbl = 0;  
        timeStampDbl  = double(curr.seconds);
        timeStampDbl -= double(init.seconds);
        timeStampDbl += (1.0e-6)*double(curr.microSeconds);
        timeStampDbl -= (1.0e-6)*double(init.microSeconds);
        return timeStampDbl;
    }

} // namespace bias


