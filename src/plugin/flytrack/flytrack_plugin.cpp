#include "flytrack_plugin.hpp"
#include <QtDebug>
#include <QMessageBox>
#include <QFileDialog>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "camera_window.hpp"
#include <iostream>
#include "video_utils.hpp"
#define _USE_MATH_DEFINES
#include <math.h>
#include "mat_to_qimage.hpp"
#include <QThreadPool> // for multithreading. Thread pool is managed by Qt
// Re-using classes for dynamic background modeling as done in video_writer_ufmf.cpp
#include "background_histogram_ufmf.hpp" //Accumulates histogram
#include "background_median_ufmf.hpp" // Computes median image 

namespace bias
{

    const QString FlyTrackPlugin::PLUGIN_NAME = QString("FlyTrack"); 
    const QString FlyTrackPlugin::PLUGIN_DISPLAY_NAME = QString("Fly Track");
    const QString FlyTrackPlugin::LOG_FILE_EXTENSION = QString("json");
    const QString FlyTrackPlugin::LOG_FILE_POSTFIX = QString("flytrack");
    const int FlyTrackPlugin::LOGGING_PRECISION = 6;

	const float FlyTrackPlugin::PI = 3.14159265358979323846;
    
	// Background estimation-related constants
    const unsigned int FlyTrackPlugin::BG_HIST_NUM_BINS = 256;
    const unsigned int FlyTrackPlugin::BG_HIST_BIN_SIZE = 1;
    const double FlyTrackPlugin::MIN_VEL_MATCH_DOTPROD = 0.25;
	const unsigned int FlyTrackPlugin::DYNAMIC_BG_MEDIAN_UPDATE_COUNT = 25; //frames to accumulate for bg comptute
	const unsigned int FlyTrackPlugin::DYNAMIC_BG_MEDIAN_UPDATE_INTERVAL = 5; //interval between bg computes
	const unsigned int FlyTrackPlugin::DYNAMIC_BG_NUM_THREADS = 2; //number of threads for bg compute


    // ROI detection-related constants    
    const unsigned int FlyTrackPlugin::fish_detect_intensity_threshold = 20;
    const unsigned int FlyTrackPlugin::fish_detect_pixel_threshold = 30;
    //const unsigned int FlyTrackPlugin::fish_size_threshold = 30;
    

    // ROIs near feeders
    const cv::Rect FlyTrackPlugin::ROI_left(90, 60, 300, 240); //(x,y,width,height)
    const cv::Rect FlyTrackPlugin::ROI_right(1440, 690, 300, 240); //(x,y,width,height)
	const float FlyTrackPlugin::roi_angle = 30.0; // Angle of the ROI that detects fish in degrees
    
    // Should background model be static or computed dynamically periodically?
	bool FlyTrackPlugin::computeBackgroundModelDynamically = true;
	unsigned int FlyTrackPlugin::backgroundUpdateFramePeriod = 0; // in frames

    
    // Public
    // ------------------------------------------------------------------------

    // FlyTrackPlugin(QWidget *parent)
    // Constructor
    // Inputs:
    // parent: parent widget
    // sets all parameters
    // initializes state
    FlyTrackPlugin::FlyTrackPlugin(QWidget *parent) : BiasPlugin(parent) 
    { 

        setupUi(this);
        initializeUi();
        connectWidgets();

        // hard code parameters
        // these should go in a config file/GUI

        //// parameters for background subtraction
        //backgroundThreshold_ = 75;
        //flyVsBgMode_ = FLY_DARKER_THAN_BG;

        // parameters for background estimation
        //nFramesBgEst_ = 100;
        //config_.bgVideoFilePath = QString("C:\\Code\\BIAS\\testdata\\20240409T155835_P1_movie1.avi");
        //config_.bgImageFilePath = QString("C:\\Code\\BIAS\\testdata\\20240409T155835_P1_movie1_bg.png");
        //config_.tmpOutDir = QString("C:\\Code\\BIAS\\testdata\\tmp");
        //config_.DEBUG = true;

        // parameters for region of interest
        //config_.roiType = CIRCLE;
        //config_.triangleCenterX = 468.6963;
        //config_.triangleCenterY = 480.2917;
        //config_.roiRadius = 428.3618;

        imwriteParams_.push_back(cv::IMWRITE_PNG_COMPRESSION);
        imwriteParams_.push_back(0);

        // parameters for resolving head/tail ambiguity
        //historyBufferLength_ = 5;
        //minVelocityMagnitude_ = 1.0; // .05; // could do this in pixels / second since we have timestamps
        //headTailWeightVelocity_ = 3.0; // weight of head-tail dot product vs previous orientation dot product

        bgImageComputed_ = false;
        trigger = false;
        fishInLeftFeeder = false;
        fishInResetROI = false;
        fishInRightFeeder = false;
        trigger_pulsed = false;
        has_triggered = true;
        active_ = false;
        lastFramePreviewed_ = -1;
        flyEllipseDequePtr_ = std::make_shared<LockableDeque<EllipseParams>>();
        initialize();


        setRequireTimer(false);

        // Initialize dynamic background threading
        dynamicBgModelingStarted_ = false;

		// Create thread pool for background reading
		bgThreadPoolPtr_ = new QThreadPool(this);
		bgThreadPoolPtr_->setMaxThreadCount(FlyTrackPlugin::DYNAMIC_BG_NUM_THREADS);

        // Create communication queues
		bgImageQueuePtr_ = std::make_shared<LockableQueue<StampedImage>>();
		bgNewDataQueuePtr_ = std::make_shared<LockableQueue<BackgroundData_ufmf>>();
		bgOldDataQueuePtr_ = std::make_shared<LockableQueue<BackgroundData_ufmf>>();
		bgMedianMatQueuePtr_ = std::make_shared<LockableQueue<cv::Mat>>();
    }

    FlyTrackPlugin::~FlyTrackPlugin()
    {
        stopDynamicBackgroundModeling();
        if (bgThreadPoolPtr_) { // waits until the workers are destroyed
            bgThreadPoolPtr_->waitForDone();
        }
    }

	void FlyTrackPlugin::startDynamicBackgroundModeling() // copied from video_writer_ufmf.cpp. Might be worth having a shared class in the future
    {
        if (dynamicBgModelingStarted_) return;
		printf("Starting dynamic backgroundee modeling threads\n");

        //Clewar queues
        bgImageQueuePtr_->clear();
		bgNewDataQueuePtr_->clear();
		bgOldDataQueuePtr_->clear();
		bgMedianMatQueuePtr_->clear();
		
		// Create histogram worker
        bgHistogramPtr_ = new BackgroundHistogram_ufmf(
            bgImageQueuePtr_,
            bgNewDataQueuePtr_,
            bgOldDataQueuePtr_,
            0 // camera number
        );
		bgHistogramPtr_->setMedianUpdateCount(FlyTrackPlugin::DYNAMIC_BG_MEDIAN_UPDATE_COUNT);
		bgHistogramPtr_->setMedianUpdateInterval(FlyTrackPlugin::DYNAMIC_BG_MEDIAN_UPDATE_INTERVAL);

        // Create median worker
        bgMedianPtr_ = new BackgroundMedian_ufmf(
            bgNewDataQueuePtr_,
            bgOldDataQueuePtr_,
            bgMedianMatQueuePtr_,
            0  // cameraNumber
        );

        // Start workers on thread pool
		bgThreadPoolPtr_->start(bgHistogramPtr_);
		bgThreadPoolPtr_->start(bgMedianPtr_);
		dynamicBgModelingStarted_ = true;
    }

	void FlyTrackPlugin::stopDynamicBackgroundModeling() // copied from video_writer_ufmf.cpp. Might be worth having a shared class in the future 
    {
        if (!dynamicBgModelingStarted_) return;

        printf("Stopping dynamic background modeling threads\n");
        fflush(stdout);

        // Tell median worker to stop
        if (!bgMedianPtr_.isNull()) {
            bgMedianPtr_->acquireLock();
            bgMedianPtr_->stop();           // Sets internal flag
            bgMedianPtr_->releaseLock();

            // Wake it up in case it's waiting on empty queue
            bgNewDataQueuePtr_->acquireLock();
            bgNewDataQueuePtr_->signalNotEmpty();
            bgNewDataQueuePtr_->releaseLock();
        }

        // Tell histogram worker to stop
        if (!bgHistogramPtr_.isNull()) {
            bgHistogramPtr_->acquireLock();
            bgHistogramPtr_->stop();
            bgHistogramPtr_->releaseLock();

            bgImageQueuePtr_->acquireLock();
            bgImageQueuePtr_->signalNotEmpty();
            bgImageQueuePtr_->releaseLock();
        }

        // Wait for both to finish
        bgThreadPoolPtr_->waitForDone();
        dynamicBgModelingStarted_ = false;
    }

    void FlyTrackPlugin::reset()
    { 
        initialize();
        openLogFile();
    }

    void FlyTrackPlugin::setFileAutoNamingString(QString autoNamingString)
    {
        fileAutoNamingString_ = autoNamingString;
    }

    void FlyTrackPlugin::setFileVersionNumber(unsigned verNum)
    {
        fileVersionNumber_ = verNum;
    }

    void FlyTrackPlugin::finishComputeBgMode() {
        printf("Computing median image\n");
        fflush(stdout);
        bgMedianImage_ = backgroundData_.getMedianImage();
        lastFrameMedianComputed_ = backgroundData_.getNFrames();
        bgImageComputed_ = true;
        printf("Saving median image to %s\n", config_.bgImageFilePath.toStdString().c_str());
        bool success = cv::imwrite(config_.bgImageFilePath.toStdString(), bgMedianImage_, imwriteParams_);
        if (!success) {
			fprintf(stderr, "Error writing background image to %s\n", config_.bgImageFilePath.toStdString().c_str());
		}
        fflush(stdout);
        FlyTrackConfig config = config_.copy();
        setFromConfig(config);
    }

    void FlyTrackPlugin::stop(){ 
        stopDynamicBackgroundModeling();
        BiasPlugin::stop();
        closeLogFile();
        if (config_.computeBgMode) {
            finishComputeBgMode();
        }
    }

    void FlyTrackPlugin::setActive(bool value)
    {
        active_ = value;
        // compute background model
        //acquireLock();
        //if (value && !bgImageComputed_) {
        //    setBackgroundModel();
        //}
        //releaseLock();
    }

    void FlyTrackPlugin::processFrames(QList<StampedImage> frameList) {
        acquireLock();
        if (config_.computeBgMode) {
            processFramesBgEstMode(frameList);
        }
        else {
            processFramesTrackMode(frameList);
        }
        releaseLock();
    }


    void FlyTrackPlugin::processFramesTrackMode(QList<StampedImage> frameList)
    { 
		static unsigned long framesPushed = 0; // This was only aded for debuggin
        StampedImage latestFrame = frameList.back();
        frameList.clear();
        currentImage_ = latestFrame.image;
        timeStamp_ = latestFrame.timeStamp;
        frameCount_ = latestFrame.frameCount;

        if (!bgImageComputed_ && !computeBackgroundModelDynamically) {
            fprintf(stderr, "Background model not computed\n");
			return;
		}


        //printf("\nProcessing frame %lu, timestamp = %f\n", frameCount_, timeStamp_);
        // empty frame
        if ((currentImage_.rows == 0) || (currentImage_.cols == 0))
        {
            fprintf(stderr, "Empty frame\n");
			return;
		}
        

        if (computeBackgroundModelDynamically) {
            // Start background modeling threads if not already started
            if (!dynamicBgModelingStarted_) {
                startDynamicBackgroundModeling();
            }

			// The following lines in the dynamic background computing block are copied from video_writer_ufmf.cpp
            // Push image to queue for background workers (non-blocking)
            bgImageQueuePtr_->acquireLock();
            if (bgImageQueuePtr_->empty()) {
                bgImageQueuePtr_->push(latestFrame);
                bgImageQueuePtr_->signalNotEmpty();
                framesPushed++;
                if (framesPushed % 2 == 0) {
					printf("Frames pushed to bg queue: %lu at frame %lu\n", framesPushed, frameCount_);
                } // TODO: Remove after debugging
            }
            bgImageQueuePtr_->releaseLock();

            
            // Check for new median image from worker (non-blocking)
            bool haveNewMedian = false;
            bgMedianMatQueuePtr_->acquireLock();
            if (!bgMedianMatQueuePtr_->empty()) {
				printf("haveNewMedian true at frame %lu\n", frameCount_); //TODO: Remove after debugging
                bgMedianImage_ = bgMedianMatQueuePtr_->front();
                bgMedianMatQueuePtr_->pop();
                haveNewMedian = true;
            }
            bgMedianMatQueuePtr_->releaseLock();
            
            // Update threshold images when new median available
            if (haveNewMedian) {
                cv::add(bgMedianImage_, config_.backgroundThreshold, bgUpperBoundImage_);
                cv::subtract(bgMedianImage_, config_.backgroundThreshold, bgLowerBoundImage_);
                bgImageComputed_ = true;
				printf("Dynamic background model updated at frame %lu\n", frameCount_); //TODO: Remove when building final
                fflush(stdout);
            }

            // If no background computed yet, skip processing this frame
            if (!bgImageComputed_) {
                return;
            }
        }

        // mismatched sizes
        if ((bgMedianImage_.rows != currentImage_.rows) || (bgMedianImage_.cols != currentImage_.cols)
            || bgMedianImage_.type() != currentImage_.type())
        {
            fprintf(stderr, "Background model and current image are not the same size\n");
            return;
        }

        // Get background/foreground membership, 255=background, 0=foreground
        backgroundSubtraction();
        
        // Dummy trigger
        
        // ROI geometry: equilateral triangle (Main ROI) engulfed by 3 outer rectangles
        // (Reset ROI), each holding 1 inner rectangle (Feeder ROI). See computeRoiGeometry().
        // Zone index 0-2 follows triangle-edge order (edge k joins vertex k and vertex k+1).
        // The geometry is static during a run, so the 7 zone masks are built once and
        // cached; setROI() invalidates them when the config changes.
        if (!roiMasksValid_ || roiTriangleMask_.size() != currentImage_.size()) {
            roiGeom_ = computeRoiGeometry(config_);
            roiTriangleMask_ = triangleMask(roiGeom_.triangle);
            for (int i = 0; i < 3; i++) {
                roiOuterMasks_[i] = rotatedRectMask(roiGeom_.outer[i]);
                roiInnerMasks_[i] = rotatedRectMask(roiGeom_.inner[i]);
            }
            roiMasksValid_ = true;
        }

        // Main ROI is the triangle; trigger = 1 when a fish IS inside it (matches the reset/feeder convention).
        // Detection results go into locals first: the HTTP server thread reads trigger and
        // outerZoneStatus_/innerZoneStatus_ without the plugin lock (runCmdFromMap), so the
        // members must never be wiped/reallocated mid-frame - they are updated element-wise
        // at the end, and a concurrent reader sees at worst the previous frame's values.
        bool triggerNow = detectFishInMask(isFg_, roiTriangleMask_);
        int outerNow[3];
        int innerNow[3];
        for (int i = 0; i < 3; i++) {
            outerNow[i] = detectFishInMask(isFg_, roiOuterMasks_[i]) ? 1 : 0;
            innerNow[i] = detectFishInMask(isFg_, roiInnerMasks_[i]) ? 1 : 0;
        }
        trigger = triggerNow;
        for (int i = 0; i < 3; i++) {
            outerZoneStatus_[i] = outerNow[i];
            innerZoneStatus_[i] = innerNow[i];
        }

        // Aggregate flags kept for backward compatibility (1 if any corresponding zone is occupied).
        fishInResetROI = (outerZoneStatus_[0] || outerZoneStatus_[1] || outerZoneStatus_[2]);
        resetStatus = fishInResetROI ? 1 : 0;
        feederStatus = (innerZoneStatus_[0] || innerZoneStatus_[1] || innerZoneStatus_[2]) ? 1 : 0;

        // Debug: print the exported status every frame, exactly as served over HTTP:
        // trigger (get-fish-status), resetROI (get-reset-status), feeder (get-feeder-status).
        printf("[ROI] frame %lu t=%.3f: trigger=%d resetROI=[%d, %d, %d] feeder=[%d, %d, %d]\n",
            frameCount_, timeStamp_, trigger ? 1 : 0,
            outerZoneStatus_[0], outerZoneStatus_[1], outerZoneStatus_[2],
            innerZoneStatus_[0], innerZoneStatus_[1], innerZoneStatus_[2]);
        fflush(stdout);
        //printf("Trigger status: %d\n", trigger);
        
        //if (fishInRightFeeder || fishInLeftFeeder) {
         //   printf("Feeder status: %d\n", feederStatus);
        //}

        
        /*
        // NOTE: trigger == 1 now means a fish IS inside the triangle; these branches act when the
        // triangle is EMPTY (all fish have left), so they test !trigger.
        if (!trigger && !trigger_pulsed) {
            trigger_pulsed = true;
            if (has_triggered) {
                printf("All fish were found outside the trigger ROI. The trigger was not reset through MATLAB. 'detect' signal will not be sent over the server\n");
                //printf("Debugging log:\n");
                //printf("trigger_pulsed: %s, has_triggered: %s, trigger: %s \n", trigger_pulsed ? "true" : "false", has_triggered ? "true" : "false", trigger ? "true" : "false");
                fflush(stdout);
            }
        }
        
        if (trigger) {
            trigger_pulsed = false;
        }

        if (!trigger && !has_triggered) {
            has_triggered = true;
            trigger_pulsed = true;
            printf("Reset command was received from the server through MATLAB. Sending 'detect' signal to the server\n");
            printf("Debugging log:\n");
            printf("All conditions were met. Trigger pulsed successfully!"); 
            fflush(stdout);
        }
        */

        //Dummy trigger ends here

        // (legacy debug rectangle draw on isFg_ removed; ROIs are drawn on the preview copy instead)



        // find connected components in isFg_
        /*
        int ccArea = largestConnectedComponent(isFg_);

        // compute mean and covariance of pixels in foreground
        flyEllipse_.frame = frameCount_;
        fitEllipse(isFg_, flyEllipse_);

        // store velocity
        updateVelocityHistory();

        // resolve head/tail ambiguity
        resolveHeadTail();

        // store ellipse
        updateEllipseHistory();

        // store orientation
        updateOrientationHistory();

        if (loggingEnabled_) {
            logCurrentFrame();
        }

        isFirst_ = false;
        */
    } 

    void FlyTrackPlugin::processFramesBgEstMode(QList<StampedImage> frameList) {

        StampedImage latestFrame = frameList.back();
        frameList.clear();
        currentImage_ = latestFrame.image;
        timeStamp_ = latestFrame.timeStamp;
        frameCount_ = latestFrame.frameCount;

        if (isFirst_) {
            backgroundData_ = BackgroundData_ufmf(latestFrame,
                FlyTrackPlugin::BG_HIST_NUM_BINS,
                FlyTrackPlugin::BG_HIST_BIN_SIZE);
            backgroundData_.addImage(latestFrame);
            lastFrameAdded_ = latestFrame.frameCount;
            nFramesAddedBgEst_ = 1;
            isFirst_ = false;
            return;
        }
        if (latestFrame.frameCount <= lastFrameAdded_ + config_.nFramesSkipBgEst) {
			return;
		}
        backgroundData_.addImage(latestFrame);
        lastFrameAdded_ += config_.nFramesSkipBgEst;
        nFramesAddedBgEst_ += 1;

    }

    /*
    void FlyTrackPlugin::getCurrentImageTrackMode(cv::Mat& currentImageCopy)
    {
        if (!bgImageComputed_) {
            currentImageCopy = currentImage_.clone();
            return;
		}
        currentImageCopy = isFg_.clone(); 
        cv::cvtColor(currentImageCopy, currentImageCopy, cv::COLOR_GRAY2BGR);
        // plot fit ellipse
        cv::ellipse(currentImageCopy, cv::Point(flyEllipse_.x, flyEllipse_.y), 
            		cv::Size(flyEllipse_.a, flyEllipse_.b), 
                    flyEllipse_.theta * 180.0 / M_PI, 
                    0, 360, cv::Scalar(0, 0, 255), 2);
        cv::Point2d head = cv::Point2d(flyEllipse_.x + flyEllipse_.a * std::cos(flyEllipse_.theta),
            			flyEllipse_.y + flyEllipse_.a * std::sin(flyEllipse_.theta));
        cv::drawMarker(currentImageCopy, head, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 10, 2);
    }
    */

    void FlyTrackPlugin::getCurrentImageTrackMode(cv::Mat& currentImageCopy)
    {
        if (!bgImageComputed_) {
            currentImageCopy = currentImage_.clone();
            return;
        }
        currentImageCopy = isFg_.clone();
        cv::cvtColor(currentImageCopy, currentImageCopy, cv::COLOR_GRAY2BGR);
        // Draw the triangle-based ROI layout: triangle (Main, red), 3 outer rects (Reset, green),
        // 3 inner rects (Feeder, blue). Colors are BGR.
        RoiGeometry geom = computeRoiGeometry(config_);
        drawTriangle(currentImageCopy, geom.triangle, cv::Scalar(0, 0, 255));
        for (int i = 0; i < 3; i++) drawRotatedRect(currentImageCopy, geom.outer[i], cv::Scalar(100, 255, 0));
        for (int i = 0; i < 3; i++) drawRotatedRect(currentImageCopy, geom.inner[i], cv::Scalar(255, 0, 0));
    }

    void FlyTrackPlugin::rotatedRectangle(cv::Mat& currentImageCopy, cv::Point2f center, cv::Size2f size, double rotationDegree, cv::Scalar color) {
		// Note: This is different from the cv::rectangle function and the cv::RotatedRect function
		cv::Point2f vertices[4];
		cv::RotatedRect rRect = cv::RotatedRect(center, size, rotationDegree);
		rRect.points(vertices);
        for (int i = 0; i < 4; i++)
            cv::line(currentImageCopy, vertices[i], vertices[(i + 1) % 4], color, 2);
    }

    void FlyTrackPlugin::rectangleUnion(cv::Mat& currentImageCopy, cv::Point2f center, cv::Size2f size, double angle, cv::Scalar color) {
        // double y_offset = std::abs(config_.roiRightFeederCenterY - config_.innerOffsetOut);
        double y_offset = std::sin(angle * PI / 180.0) * size.height;
        rotatedRectangle(currentImageCopy, center, size, angle, color);
		rotatedRectangle(currentImageCopy, cv::Point2f(center.x, center.y + y_offset), size, -angle, color);
    }

    void FlyTrackPlugin::getCurrentImageComputeBgMode(cv::Mat& currentImageCopy)
    {
        if (isFirst_) {
            currentImageCopy = currentImage_.clone();
            return;
        }
        if (backgroundData_.getNFrames() == lastFrameMedianComputed_) {
            currentImageCopy = lastImagePreviewed_;
            return;
        }
        currentImageCopy = backgroundData_.getMedianImage().clone();
        lastFrameMedianComputed_ = backgroundData_.getNFrames();
        cv::cvtColor(currentImageCopy, currentImageCopy, cv::COLOR_GRAY2BGR);

        // add text
        std::stringstream statusStream;
        statusStream << "N Frames added: " << nFramesAddedBgEst_ << ", Last frame added: " << lastFrameAdded_;
        double fontScale = 1.0;
        int thickness = 2;
        int baseline = 0;

        //cv::Size textSize = cv::getTextSize(foundStream.str(), CV_FONT_HERSHEY_SIMPLEX, fontScale, thickness, &baseline);
        cv::Size textSize = cv::getTextSize(statusStream.str(), cv::FONT_HERSHEY_SIMPLEX, fontScale, thickness, &baseline);
        cv::Point textPoint(currentImageCopy.cols / 2 - textSize.width / 2, textSize.height + baseline);
        //cv::putText(currentImageBGR, foundStream.str(), textPoint, CV_FONT_HERSHEY_SIMPLEX, fontScale, boxColor,thickness);
        cv::putText(currentImageCopy, statusStream.str(), textPoint, 
            cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0, 0, 255),thickness);

    }

    cv::Mat FlyTrackPlugin::getCurrentImage() {
        acquireLock();
        if (frameCount_ == lastFramePreviewed_) {
            releaseLock();
            return lastImagePreviewed_;
        }
        cv::Mat currentImageCopy;
        if (config_.computeBgMode) {
            getCurrentImageComputeBgMode(currentImageCopy);
        }
        else {
            getCurrentImageTrackMode(currentImageCopy);
        }
        lastFramePreviewed_ = frameCount_;
        lastImagePreviewed_ = currentImageCopy;
        releaseLock();
        return currentImageCopy;
    }

    QString FlyTrackPlugin::getName()
    {
        return PLUGIN_NAME;
    }


    QString FlyTrackPlugin::getDisplayName()
    {
        return PLUGIN_DISPLAY_NAME;
    }


    QPointer<CameraWindow> FlyTrackPlugin::getCameraWindow()
    {
        QPointer<CameraWindow> cameraWindowPtr = (CameraWindow*)(parent());
        return cameraWindowPtr;
    }


    RtnStatus FlyTrackPlugin::runCmdFromMap(QVariantMap cmdMap, bool showErrorDlg, QString& value)
    {
        RtnStatus rtnStatus;
        rtnStatus.success = true;
        rtnStatus.message = QString("");

        QString errMsgTitle("Plugin runCmdFromMap Error");

        if (!cmdMap.contains("cmd"))
        {
            QString errMsgText("FlyTrackPlugin::runPluginCmd: cmd not found in map");
            if (showErrorDlg)
            {
                QMessageBox::critical(this, errMsgTitle, errMsgText);
            }
            rtnStatus.success = false;
            rtnStatus.message = errMsgText;
            return rtnStatus;
        }
        if (!cmdMap["cmd"].canConvert<QString>())
        {
            QString errMsgText("FlyTrackPlugin::runPluginCmd: unable to convert plugin name to string");
            if (showErrorDlg)
            {
                QMessageBox::critical(this, errMsgTitle, errMsgText);
            }
            rtnStatus.success = false;
            rtnStatus.message = errMsgText;
            return rtnStatus;
        }
        QString cmd = cmdMap["cmd"].toString();

        if (cmd == QString("pop-front-track"))
        {
            EllipseParams ell;
            rtnStatus = popFrontTrack(ell);
            if (rtnStatus.success) {
				value = ellipseToJson(ell);
            }
        }
        else if (cmd == QString("get-fish-status")) {
            //value = fishStatusToJson(has_triggered);
			value = fishStatusToJson(trigger);
        }
        else if (cmd == QString("reset-fish-trigger")) {
            value = fishStatusToJson(has_triggered);
            has_triggered = false;
        }
        else if (cmd == QString("get-reset-status")) {
            value = resetStatusToJson(outerZoneStatus_);
        }
        else if (cmd == QString("get-feeder-status")) {
            value = feederStatusToJson(innerZoneStatus_);
        }
        else if (cmd == QString("get-fish-centroids")) {
            if (fishCentroids_.size() == 0) {
                value = QString("[]");
            }
            else {
				value = fishCentroidsToJson(fishCentroids_);
            }
        }
        else if (cmd == QString("pop-back-track"))
        {
			EllipseParams ell;
			rtnStatus = popBackTrack(ell);
			if (rtnStatus.success) {
                value = ellipseToJson(ell);
            }
        }
		else if (cmd == QString("get-last-clear-track"))
		{
			EllipseParams ell;
			rtnStatus = getLastClearTrack(ell);
			if (rtnStatus.success) {
				value = ellipseToJson(ell);
			}
		}
        else if (cmd == QString("get-arena-params"))
		{
            EllipseParams ell;
			rtnStatus = getArenaParams(ell);
            if (rtnStatus.success) {
                value = ellipseToJson(ell);
            }
        }
        else
        {
            QString errMsgText = QString("FlyTrackPlugin::runPluginCmd: unknown cmd %1").arg(cmd);
            if (showErrorDlg)
            {
                QMessageBox::critical(this, errMsgTitle, errMsgText);
            }
            rtnStatus.success = false;
            rtnStatus.message = errMsgText;
        }

        return rtnStatus;
    }

    RtnStatus FlyTrackPlugin::popFrontTrack(EllipseParams& ell) {
        RtnStatus rtnStatus;
		rtnStatus.success = false;
		rtnStatus.message = QString("");
        if (flyEllipseDequePtr_ == NULL) {
            rtnStatus.message = QString("Ellipse queue not allocated");
            return rtnStatus;
        }
        flyEllipseDequePtr_->acquireLock();
        if (flyEllipseDequePtr_->empty()) {
			rtnStatus.message = QString("Ellipse queue empty");
		}
        else {
            ell = flyEllipseDequePtr_->front();
            flyEllipseDequePtr_->pop_front();
            rtnStatus.success = true;
        }
        flyEllipseDequePtr_->releaseLock();
		return rtnStatus;
	
    }

    RtnStatus FlyTrackPlugin::popBackTrack(EllipseParams& ell) {
        RtnStatus rtnStatus;
        rtnStatus.success = false;
        rtnStatus.message = QString("");
        if (flyEllipseDequePtr_ == NULL) {
            rtnStatus.message = QString("Ellipse queue not allocated");
            return rtnStatus;
        }
        flyEllipseDequePtr_->acquireLock();
        if (flyEllipseDequePtr_->empty()) {
            rtnStatus.message = QString("Ellipse queue empty");
        }
        else {
            ell = flyEllipseDequePtr_->back();
            flyEllipseDequePtr_->pop_back();
            rtnStatus.success = true;
        }
        flyEllipseDequePtr_->releaseLock();
        return rtnStatus;

    }

    RtnStatus FlyTrackPlugin::getLastClearTrack(EllipseParams& ell) {
        RtnStatus rtnStatus;
        rtnStatus.success = false;
        rtnStatus.message = QString("");
        if (flyEllipseDequePtr_ == NULL) {
            rtnStatus.message = QString("Ellipse queue not allocated");
            return rtnStatus;
        }
        flyEllipseDequePtr_->acquireLock();
        if (flyEllipseDequePtr_->empty()) {
            rtnStatus.message = QString("Ellipse queue empty");
        }
        else {
            ell = flyEllipseDequePtr_->back();
			flyEllipseDequePtr_->clear();
            rtnStatus.success = true;
        }
        flyEllipseDequePtr_->releaseLock();
        return rtnStatus;

    }

    RtnStatus FlyTrackPlugin::getArenaParams(EllipseParams& ell) {
        RtnStatus rtnStatus;
        rtnStatus.success = true;
        rtnStatus.message = QString("");
        //if (config_.roiType == NONE) return rtnStatus;
		//ell.x = config_.triangleCenterX;
		//ell.y = config_.triangleCenterY;
		//ell.a = config_.roiRadius;
		//ell.b = config_.roiRadius;
		//ell.theta = 0.0;
        return rtnStatus;
    }

    QVariantMap FlyTrackPlugin::getConfigAsMap()  
    {
        QVariantMap configMap = config_.toMap();
        return configMap;
    }

    void FlyTrackPlugin::setRoiUIValues() {
		// This is where the ROI values in the GUI are set to the values in the config file
        roiTypeComboBox->setCurrentIndex(config_.roiType);
        triangleCenterXSpinBox->setValue(config_.triangleCenterX);
        triangleCenterYSpinBox->setValue(config_.triangleCenterY);
        triangleSideSpinBox->setValue(config_.triangleSide);
        triangleAngleSpinBox->setValue(config_.triangleAngle);
        innerWidthSpinBox->setValue(config_.innerWidth);
        innerHeightSpinBox->setValue(config_.innerHeight);
		roiRightFeederWidthSpinBox->setValue(config_.roiRightFeederWidth);
		roiRightFeederHeightSpinBox->setValue(config_.roiRightFeederHeight);
		roiResetCenterXSpinBox->setValue(config_.roiResetCenterX);
		roiResetCenterYSpinBox->setValue(config_.roiResetCenterY);
		roiResetWidthSpinBox->setValue(config_.roiResetWidth);
		outerDepthSpinBox->setValue(config_.outerDepth);
        innerOffsetAlongSpinBox->setValue(config_.innerOffsetAlong);
        innerOffsetOutSpinBox->setValue(config_.innerOffsetOut);
        roiRightFeederCenterXSpinBox->setValue(config_.roiRightFeederCenterX);
        roiRightFeederCenterYSpinBox->setValue(config_.roiRightFeederCenterY);

        triangleCenterXSpinBox->setEnabled(true);
        triangleCenterYSpinBox->setEnabled(true);
        triangleSideSpinBox->setEnabled(true);
        triangleAngleSpinBox->setEnabled(true);
		innerWidthSpinBox->setEnabled(true);
		innerHeightSpinBox->setEnabled(true);
		roiRightFeederWidthSpinBox->setEnabled(true);
		roiRightFeederHeightSpinBox->setEnabled(true);
		roiResetCenterXSpinBox->setEnabled(true);
		roiResetCenterYSpinBox->setEnabled(true);
		roiResetWidthSpinBox->setEnabled(true);
		outerDepthSpinBox->setEnabled(true);

    }

    void FlyTrackPlugin::connectWidgets()
    {
        connect(
            donePushButton,
            SIGNAL(clicked()),
            this,
            SLOT(donePushButtonClicked())
        );

        connect(
            applyPushButton,
            SIGNAL(clicked()),
            this,
            SLOT(applyPushButtonClicked())
        );

        connect(
            cancelPushButton,
            SIGNAL(clicked()),
            this,
            SLOT(cancelPushButtonClicked())
        );

        connect(
            loadBgPushButton,
            SIGNAL(clicked()),
            this,
            SLOT(loadBgPushButtonClicked())
        );

        connect(
            triangleCenterXSpinBox,
            SIGNAL(valueChanged(int)),
            this,
            SLOT(roiUiChanged(int))
        );
        connect(
            triangleCenterYSpinBox,
            SIGNAL(valueChanged(int)),
            this,
            SLOT(roiUiChanged(int))
        );
        connect(
            roiTypeComboBox,
            SIGNAL(activated(int)),
            this,
            SLOT(roiUiChanged(int))
        );
        connect(
            bgImageFilePathToolButton,
            SIGNAL(clicked()),
            this,
            SLOT(bgImageFilePathToolButtonClicked())
        );
        connect(
            logFilePathToolButton,
            SIGNAL(clicked()),
            this,
            SLOT(logFilePathToolButtonClicked())
        );
        connect(
			tmpOutDirToolButton,
			SIGNAL(clicked()),
			this,
			SLOT(tmpOutDirToolButtonClicked())
		);
        connect(
            computeBgModeComboBox,
            SIGNAL(activated(int)),
            this,
            SLOT(computeBgModeComboBoxChanged())
        );
    }

    void FlyTrackPlugin::showEvent(QShowEvent* event) {
        QWidget::showEvent(event);
        setFromConfig(config_);
    }

    void FlyTrackPlugin::donePushButtonClicked() {
        try {
            applyPushButtonClicked();
            // close the dialog
            close();
        }
        catch (std::exception& e) {
            fflush(stdout);
			fprintf(stderr, "Error closing dialog: %s\n", e.what());
		}
    }
    void FlyTrackPlugin::cancelPushButtonClicked() {
        try {
            close();
        }
        catch (std::exception& e) {
            fflush(stdout);
            fprintf(stderr, "Error closing dialog: %s\n", e.what());
        }
    }

    void FlyTrackPlugin::applyPushButtonClicked() {
        try {
            FlyTrackConfig config;
            RtnStatus rtnStatus;
            rtnStatus.success = true;
            rtnStatus.message = QString("");
            getUiValues(config);
            if (rtnStatus.success) {
                rtnStatus = setFromConfig(config);
                if (rtnStatus.success) {
                }
                else {
                    QMessageBox::critical(this, QString("Error setting config values"), rtnStatus.message);
                }
            }
            else {
                QMessageBox::critical(this, QString("Error getting config values"), rtnStatus.message);
            }
            fflush(stdout);
        }
		catch (std::exception& e) {
			fflush(stdout);
			fprintf(stderr, "Error applying settings: %s\n", e.what());
		}
    }

    void FlyTrackPlugin::bgImageFilePathToolButtonClicked() {
        try {
            QString bgImageFilePath = bgImageFilePathLineEdit->text();
            QString bgImageDir = QFileInfo(bgImageFilePath).absoluteDir().absolutePath();
            bgImageFilePath = QFileDialog::getSaveFileName(this, "Select Background Image File",
                bgImageDir, "Image Files (*.png *.jpg *.bmp)", NULL, QFileDialog::DontConfirmOverwrite);
            if (bgImageFilePath.isEmpty()) {
                fprintf(stderr, "No background image selected\n");
                return;
            }
            bgImageFilePathLineEdit->setText(bgImageFilePath);
        }
		catch (std::exception& e) {
			fflush(stdout);
			fprintf(stderr, "Error selecting background image file: %s\n", e.what());
		}
    }

    //void FlyTrackPlugin::bgVideoFilePathToolButtonClicked() {
    //    QString bgVideoFilePath = bgVideoFilePathLineEdit->text();
    //    QString bgVideoDir = QFileInfo(bgVideoFilePath).absoluteDir().absolutePath();
    //    bgVideoFilePath = QFileDialog::getOpenFileName(this, "Select Video to compute background from",
    //        bgVideoDir, "Video Files (*.avi *.ufmf *.fmf *mp4)");
    //    if (bgVideoFilePath.isEmpty()) {
    //        fprintf(stderr,"No background video selected\n");
    //        //return;
    //    }
    //    bgVideoFilePathLineEdit->setText(bgVideoFilePath);
    //}

    void FlyTrackPlugin::logFilePathToolButtonClicked() {
        try {
            QString logFilePath = logFilePathLineEdit->text();
            QString logFileDir = QFileInfo(logFilePath).absoluteDir().absolutePath();
            logFilePath = QFileDialog::getSaveFileName(this, "Output track file", logFileDir, "JSON Files (*." + LOG_FILE_EXTENSION + ")");
            if (logFilePath.isEmpty()) {
                fprintf(stderr, "No output file selected\n");
                return;
            }
            logFilePathLineEdit->setText(logFilePath);
        }
		catch (std::exception& e) {
			fflush(stdout);
			fprintf(stderr, "Error selecting output file: %s\n", e.what());
		}
    }

    void FlyTrackPlugin::tmpOutDirToolButtonClicked() {
        try {
            QString tmpOutDir = tmpOutDirLineEdit->text();
            tmpOutDir = QFileDialog::getExistingDirectory(this, "Debug output folder", tmpOutDir);
            if (tmpOutDir.isEmpty()) {
                fprintf(stderr, "No output directory selected\n");
                return;
            }
            tmpOutDirLineEdit->setText(tmpOutDir);
        }
		catch (std::exception& e) {
			fflush(stdout);
			fprintf(stderr, "Error selecting debug output directory: %s\n", e.what());
		}
    }

    void FlyTrackPlugin::computeBgModeComboBoxChanged() {
        try {
            setUiEnabled();
        }
		catch (std::exception& e) {
			fflush(stdout);
            fprintf(stderr, "Error changing compute background mode: %s\n", e.what());
		}
	}

    void FlyTrackPlugin::roiUiChanged(int v) {
        try {
            FlyTrackConfig roiConfig = config_.copy();
            getUiRoiValues(roiConfig);
            setPreviewImage(bgMedianImage_, roiConfig);
            setUiEnabled();
        }
        catch (std::exception& e) {
            fflush(stdout);
			fprintf(stderr, "Error changing ROI parameters: %s\n", e.what());
        }
    }

    void FlyTrackPlugin::loadBgPushButtonClicked() {
        try {
            FlyTrackConfig bgEstConfig = config_.copy();
            getUiBgEstValues(bgEstConfig);
            bool success = setBgImageFilePath(bgEstConfig.bgImageFilePath);
            if (!success) {
                // || checkFileExists(bgEstConfig.bgVideoFilePath))) {
                QMessageBox::critical(this, QString("Error loading background model"),
                    QString("Could not load background image from file %1.").arg(bgEstConfig.bgImageFilePath));
            }
        }
		catch (std::exception& e) {
			fflush(stdout);
			fprintf(stderr, "Error loading background model: %s\n", e.what());
		}
    }

    RtnStatus FlyTrackPlugin::setFromConfig(FlyTrackConfig config)
	{
		RtnStatus rtnStatus;
        rtnStatus.success = true;
        rtnStatus.message = QString("");
        try {

            //printf("Setting config:\n");

            config_ = config;
            setBgImageFilePath(config_.bgImageFilePath);
            setROI(config);

            if (config_.computeBgMode) {
                computeBgModeComboBox->setCurrentIndex(0);
            }
            else {
                computeBgModeComboBox->setCurrentIndex(1);
            }
            bgImageFilePathLineEdit->setText(config_.bgImageFilePath);
            nFramesSkipLineEdit->setText(QString::number(config_.nFramesSkipBgEst));
            flyVsBgModeComboBox->setCurrentIndex(config_.flyVsBgMode);
            backgroundThresholdLineEdit->setText(QString::number(config_.backgroundThreshold));
            setRoiUIValues();
            logFilePathLineEdit->setText(config_.tmpTrackFilePath);
            logFileNameLineEdit->setText(config_.trackFileName);
            tmpOutDirLineEdit->setText(config_.tmpOutDir);
            DEBUGCheckBox->setChecked(config_.DEBUG);

            //config_.print();
            setUiEnabled();
        }
		catch (std::exception& e) {
			fflush(stdout);
			fprintf(stderr, "Error setting config: %s\n", e.what());
            rtnStatus.success = false;
			rtnStatus.message = QString(e.what());
		}

		return rtnStatus;
	}

    bool FlyTrackPlugin::saveBgMedianImage(cv::Mat bgMedianImage, QString bgImageFilePath) {
        try {
            printf("Saving median image to %s\n", bgImageFilePath.toStdString().c_str());
            bool success = cv::imwrite(bgImageFilePath.toStdString(), bgMedianImage, imwriteParams_);
            if (success) printf("Done\n");
            else fprintf(stderr, "Failed to write background median image to %s\n", bgImageFilePath.toStdString().c_str());
            return success;
        }
		catch (std::exception& e) {
			fflush(stdout);
			fprintf(stderr, "Error saving background median image: %s\n", e.what());
            return false;
		}
    }

    bool FlyTrackPlugin::setBgImageFilePath(QString newBgImageFilePath) {

        cv::Mat bgMedianImage;
        bool success = loadBackgroundModel(newBgImageFilePath, bgMedianImage);
        if (!success) return false;
        setBackgroundModel(bgMedianImage, config_);
        config_.bgImageFilePath = newBgImageFilePath;
        bgImageComputed_ = true;
        return true;
    }

    void FlyTrackPlugin::getUiRoiValues(FlyTrackConfig& config) {
        ROIType roiType = (ROIType)roiTypeComboBox->currentIndex();
        double triangleCenterX = triangleCenterXSpinBox->value();
        double triangleCenterY = triangleCenterYSpinBox->value();
        double triangleSide = triangleSideSpinBox->value();
        double triangleAngle = triangleAngleSpinBox->value();
		double roiResetCenterX = roiResetCenterXSpinBox->value();
		double roiResetCenterY = roiResetCenterYSpinBox->value();
		double roiResetWidth = roiResetWidthSpinBox->value();
		double outerDepth = outerDepthSpinBox->value();
		double innerOffsetAlong = innerOffsetAlongSpinBox->value();
		double innerOffsetOut = innerOffsetOutSpinBox->value();
		double innerWidth = innerWidthSpinBox->value();
		double innerHeight = innerHeightSpinBox->value();
		double roiRightFeederCenterX = roiRightFeederCenterXSpinBox->value();
		double roiRightFeederCenterY = roiRightFeederCenterYSpinBox->value();
		double roiRightFeederWidth = roiRightFeederWidthSpinBox->value();
		double roiRightFeederHeight = roiRightFeederHeightSpinBox->value();
		config.setRoiParams(roiType, triangleCenterX, triangleCenterY, triangleSide, triangleAngle, innerOffsetAlong, innerOffsetOut, innerWidth, innerHeight, roiRightFeederCenterX, roiRightFeederCenterY, roiRightFeederWidth, roiRightFeederHeight, roiResetCenterX, roiResetCenterY, roiResetWidth, outerDepth);
    }

    void FlyTrackPlugin::getUiBgEstValues(FlyTrackConfig& config) {
        config.computeBgMode = computeBgModeComboBox->currentIndex() == 0;
        config.bgImageFilePath = bgImageFilePathLineEdit->text();
        config.nFramesSkipBgEst = nFramesSkipLineEdit->text().toInt();
    }

    void FlyTrackPlugin::getUiValues(FlyTrackConfig& config) {
        try {
            getUiBgEstValues(config);
            config.flyVsBgMode = (FlyVsBgModeType)flyVsBgModeComboBox->currentIndex();
            config.backgroundThreshold = backgroundThresholdLineEdit->text().toInt();
            getUiRoiValues(config);
            config.tmpOutDir = tmpOutDirLineEdit->text();
            config.DEBUG = DEBUGCheckBox->isChecked();
            config.tmpTrackFilePath = logFilePathLineEdit->text();
            config.trackFileName = logFileNameLineEdit->text();
        }
        catch (std::exception& e) {
            fflush(stdout);
			fprintf(stderr,"Error getting UI values: %s\n", e.what());
		}
	}

    void FlyTrackPlugin::setUiEnabled() {
        FlyTrackConfig config;
        getUiValues(config);
        bool v = config.computeBgMode;
        bgImageFilePathLineEdit->setEnabled(true);
        bgImageFilePathLabel->setEnabled(true);
    	nFramesSkipLineEdit->setEnabled(v);
        nFramesSkipLabel->setEnabled(v);
	    loadBgPushButton->setEnabled(!v);

        flyVsBgModeComboBox->setEnabled(!v);
        flyVsBgModeLabel->setEnabled(!v);
        backgroundThresholdLineEdit->setEnabled(!v);
        backgroundThresholdLabel->setEnabled(!v);
        roiTypeComboBox->setEnabled(!v);
        roiTypeLabel->setEnabled(!v);
        logFilePathLineEdit->setEnabled(!v);
        logFilePathLabel->setEnabled(!v);

        roiTypeComboBox->setEnabled(!v);
        roiTypeLabel->setEnabled(!v);
        switch (config.roiType) {
            case RECTANGLE:
                triangleCenterXSpinBox->setEnabled(!v);
                triangleCenterXLabel->setEnabled(!v);
                triangleCenterYSpinBox->setEnabled(!v);
                triangleCenterYLabel->setEnabled(!v);
                triangleSideSpinBox->setEnabled(!v);
                triangleSideLabel->setEnabled(!v);
                triangleAngleSpinBox->setEnabled(!v);
                triangleAngleLabel->setEnabled(!v);
                innerOffsetAlongSpinBox->setEnabled(!v);
				innerOffsetAlongLabel->setEnabled(!v);
				innerOffsetOutSpinBox->setEnabled(!v);
				innerOffsetOutLabel->setEnabled(!v);
				roiRightFeederCenterXSpinBox->setEnabled(!v);
				roiRightFeederCenterXLabel->setEnabled(!v);
				innerWidthLabel->setEnabled(!v);
				innerWidthSpinBox->setEnabled(!v);
				innerHeightLabel->setEnabled(!v);
				innerHeightSpinBox->setEnabled(!v);
				roiRightFeederCenterYSpinBox->setEnabled(!v);
				roiRightFeederCenterYLabel->setEnabled(!v);
				roiRightFeederWidthLabel->setEnabled(!v);
				roiRightFeederWidthSpinBox->setEnabled(!v);
				roiRightFeederHeightLabel->setEnabled(!v); 
				roiRightFeederHeightSpinBox->setEnabled(!v);
				roiResetCenterXLabel->setEnabled(!v);
				roiResetCenterXSpinBox->setEnabled(!v);
				roiResetCenterYLabel->setEnabled(!v);
				roiResetCenterYSpinBox->setEnabled(!v);
				roiResetWidthLabel->setEnabled(!v);
				roiResetWidthSpinBox->setEnabled(!v);
				outerDepthLabel->setEnabled(!v);
				outerDepthSpinBox->setEnabled(!v);
                break;
        }
		tmpOutDirLineEdit->setEnabled(true);
        tmpOutDirLabel->setEnabled(true);
		DEBUGCheckBox->setEnabled(true);
    }

	RtnStatus FlyTrackPlugin::setConfigFromMap(QVariantMap configMap)
	{
		FlyTrackConfig config;
		RtnStatus rtnStatus = config.fromMap(configMap);
        if (rtnStatus.success)
        {
			rtnStatus = setFromConfig(config);
		}
		return rtnStatus;
	}

	RtnStatus FlyTrackPlugin::setConfigFromJson(QByteArray jsonArray)
	{
		FlyTrackConfig config;
		RtnStatus rtnStatus = config.fromJson(jsonArray);
        if (rtnStatus.success)
        {
			rtnStatus = setFromConfig(config);
		}
		return rtnStatus;
    }

    bool FlyTrackPlugin::pluginsEnabled()
    {
        return getCameraWindow() -> isPluginEnabled();
    }


    void FlyTrackPlugin::setPluginsEnabled(bool value)
    {
        getCameraWindow() -> setPluginEnabled(value);
    }


    QString FlyTrackPlugin::getLogFileExtension()
    {
        return LOG_FILE_EXTENSION;
    }

    QString FlyTrackPlugin::getLogFilePostfix()
    {
        return LOG_FILE_POSTFIX;
    }

    QString FlyTrackPlugin::getLogFileName(bool includeAutoNaming)
    {
        QString logFileName;
        if (config_.trackFileNameSet()) {
            logFileName = config_.trackFileName;
        }
        else{
            QPointer<CameraWindow> cameraWindowPtr = getCameraWindow();
            logFileName = cameraWindowPtr->getVideoFileName() + QString("_") + getLogFilePostfix();
        }
        if (includeAutoNaming)
        {
            if (!fileAutoNamingString_.isEmpty())
            {
                logFileName += QString("_") + fileAutoNamingString_;
            }
            if (fileVersionNumber_ != 0)
            {
                QString verStr = QString("_v%1").arg(fileVersionNumber_,3,10,QChar('0'));
                logFileName += verStr;
            }
        }
        logFileName += QString(".") + getLogFileExtension();
        return logFileName;
    }


    QString FlyTrackPlugin::getLogFileFullPath(bool includeAutoNaming)
    {
        if (config_.trackFilePathSet()) {
            return config_.tmpTrackFilePath;
        }
        QString logFileName = getLogFileName(includeAutoNaming);
        QPointer<CameraWindow> cameraWindowPtr = getCameraWindow();
        logFileDir_ = cameraWindowPtr -> getVideoFileDir();
        QString logFileFullPath = logFileDir_.absoluteFilePath(logFileName);
        return logFileFullPath;
    }

    // Protected methods
    // ------------------------------------------------------------------------

    void FlyTrackPlugin::setRequireTimer(bool value)
    {
        requireTimer_ = value;
    }


    void FlyTrackPlugin::openLogFile()
    {
        loggingEnabled_ = getCameraWindow() -> isLoggingEnabled();
        if ((config_.computeBgMode == false) && loggingEnabled_)
        {
            QString logFileFullPath = getLogFileFullPath(true);
            qDebug() << logFileFullPath;
            fprintf(stderr,"Outputting trajectory to file: %s",logFileFullPath.toStdString().c_str());
            logFile_.setFileName(logFileFullPath);
            bool isOpen = logFile_.open(QIODevice::WriteOnly | QIODevice::Text);
            if (isOpen)
            {
                logStream_.setDevice(&logFile_);
                logStream_.setRealNumberNotation(QTextStream::ScientificNotation);
                logStream_.setRealNumberPrecision(FlyTrackPlugin::LOGGING_PRECISION);
                logStream_ << "{\n  \"track\": [\n";
            }
            else
            {
				fprintf(stderr,"Failed to open log file: %s\n",logFileFullPath.toStdString().c_str());
                loggingEnabled_ = false;
			}
        }
    }

    void FlyTrackPlugin::closeLogFile()
    {
        if (loggingEnabled_ && (config_.computeBgMode == false) && logFile_.isOpen())
        {
            logStream_ << "\n  ]\n}";
            logStream_.flush();
            logFile_.close();
        }
    }

    // Protected
    // ------------------------------------------------------------------------

    // void initialize()
    // (re-)initialize state
    void FlyTrackPlugin::initialize() {
        isFirst_ = true;
        meanFlyVelocity_ = cv::Point2d(0.0, 0.0);
        meanFlyOrientation_ = 0.0;
        flyEllipseHistory_.clear();
        flyEllipseDequePtr_->acquireLock();
        flyEllipseDequePtr_->clear();
        flyEllipseDequePtr_->releaseLock();
        velocityHistory_.clear();
        orientationHistory_.clear();
        headTailResolved_ = false;
        outerZoneStatus_ = std::vector<int>(3, 0);
        innerZoneStatus_ = std::vector<int>(3, 0);
        roiMasksValid_ = false;
        roiOuterMasks_ = std::vector<cv::Mat>(3);
        roiInnerMasks_ = std::vector<cv::Mat>(3);

        setFromConfig(config_);
    }

    void FlyTrackPlugin::initializeUi() {

        // set items in ROI combobox to match order of enum
        roiTypeComboBox->clear();
        QString s;
        for(int i=0; i<N_ROI_TYPES; i++){
			roiTypeToString((ROIType)i, s);
			roiTypeComboBox->addItem(s, i);
		}
        // set items in flyVsBgMode combobox to match order of enum
        flyVsBgModeComboBox->clear();
        for (int i = 0; i < N_FLY_VS_BG_MODES; i++) {
            flyVsBgModeToString((FlyVsBgModeType)i, s);
            flyVsBgModeComboBox->addItem(s, i);
        }

        previewImageLabel->setBackgroundRole(QPalette::Base);
        previewImageLabel->setScaledContents(true);

    }


    // cv::Mat circleROI(double centerX, double centerY, double centerRadius)
    // create a circular region of interest mask, inside is 255, outside 0
    // inputs:
    // centerX, centerY: center of circle
    // centerRadius: radius of circle
    // returns: mask image
    cv::Mat FlyTrackPlugin::circleROI(double centerX, double centerY, double centerRadius) {
        cv::Mat mask = cv::Mat::zeros(bgMedianImage_.size(), CV_8U);
        cv::circle(mask, cv::Point(centerX, centerY), centerRadius, cv::Scalar(255), -1);
        return mask;
    }

    // Create a rectangular mask (boolean), inside is 255, outside is 0
    // Returns mask
    cv::Mat FlyTrackPlugin::rectangleROI(double centerX, double centerY, double width, double height) {
        cv::Mat mask = cv::Mat::zeros(bgMedianImage_.size(), CV_8U);
        cv::rectangle(mask, cv::Rect(centerX, centerY, width, height), cv::Scalar(255), -1);
        return mask;
    }

    cv::Mat FlyTrackPlugin::rotatedRectangleROI(double centerX, double centerY, double width, double height, double angle) {
		cv::Mat mask = cv::Mat::zeros(bgMedianImage_.size(), CV_8U);
		cv::RotatedRect rotatedRectangle(cv::Point2f(centerX, centerY), cv::Size2f(width, height), angle);
		cv::Point2f vertices[4];
		rotatedRectangle.points(vertices);
		cv::Point points[4];
        for (int i = 0; i < 4; i++) {
            points[i] = vertices[i];
        }
        cv::fillConvexPoly(mask,
            points,
            4,
			cv::Scalar(255)); 
        return mask;
    }

    cv::Mat FlyTrackPlugin::rotatedUnionROI(double centerX, double centerY, double width, double height, double angle) {
        cv::Mat unionMask = cv::Mat::zeros(bgMedianImage_.size(), CV_8U);
        cv::Mat mask1 = rotatedRectangleROI(centerX, centerY, width, height, angle);
        double y_offset = std::sin(angle * PI / 180.0) * height;
        // double y_offset = std::abs(config_.innerOffsetOut - config_.roiRightFeederCenterY);
        cv::Mat mask2 = rotatedRectangleROI(centerX, centerY + y_offset, width, height, -angle);
        cv::bitwise_or(mask1, mask2, unionMask);
        return unionMask;
        //cv::polylines(image, &pts, &n, 1, true, cv::Scalar(0, 0, 255), 2); // Blue polygon for union
    }

    // RoiGeometry computeRoiGeometry(config)
    // Build the triangle-based ROI layout from the config. Mapping of the (unchanged) GUI fields:
    //   Main ROI:   triangleCenterX/Y = triangle center, triangleSide = triangle side length,
    //               triangleAngle = rotation of the whole structure (degrees).
    //   Reset ROI:  outer-rect extent along the shared edge is hard-coded to the triangle side
    //               (triangleSide), so the rectangles always meet at the triangle vertices;
    //               outerDepth = outward depth. (roiResetWidth is no longer used.)
    //   Feeder ROI (Left): innerOffsetAlong/Y = inner-rect center offset, measured from the outer
    //               rectangle's corner at the edge-start vertex, along the edge (X) and outward (Y);
    //               innerWidth/Height = inner-rect size.
    // All three outer rectangles are identical (from the Reset ROI); all three inner rectangles are
    // identical (from the Feeder-Left ROI). Zone index 0-2 = triangle edge k (joining vertex k, k+1).
    RoiGeometry FlyTrackPlugin::computeRoiGeometry(const FlyTrackConfig& config) {
        RoiGeometry geom;
        const double cx = config.triangleCenterX;
        const double cy = config.triangleCenterY;
        const double side = config.triangleSide;        // triangle side length
        const double angleDeg = config.triangleAngle;   // rotation of the whole structure (degrees)
        const double W = side;                      // outer-rect extent along the edge, hard-coded to the triangle side
        const double H = config.outerDepth;     // outer-rect extent outward (depth)
        const double fx = config.innerOffsetAlong;  // inner center offset along edge (u)
        const double fy = config.innerOffsetOut;  // inner center offset outward (n)
        const double fw = config.innerWidth;    // inner-rect width  (along edge)
        const double fh = config.innerHeight;   // inner-rect height (outward)

        const double R = side / std::sqrt(3.0);     // circumradius (center to vertex)
        const double phi = angleDeg * PI / 180.0;
        const double base = -PI / 2.0;              // at angle 0, vertex 0 points "up" (image y-down)

        // Triangle vertices
        for (int k = 0; k < 3; k++) {
            double a = phi + base + k * 2.0 * PI / 3.0;
            geom.triangle.push_back(cv::Point2f((float)(cx + R * std::cos(a)),
                                                (float)(cy + R * std::sin(a))));
        }

        // One outer rectangle + one inner rectangle per triangle edge
        for (int k = 0; k < 3; k++) {
            cv::Point2f Vk = geom.triangle[k];
            cv::Point2f Vk1 = geom.triangle[(k + 1) % 3];
            double Mx = 0.5 * (Vk.x + Vk1.x);       // edge midpoint
            double My = 0.5 * (Vk.y + Vk1.y);
            double ex = Vk1.x - Vk.x, ey = Vk1.y - Vk.y;
            double elen = std::sqrt(ex * ex + ey * ey);
            if (elen < 1e-9) elen = 1.0;            // guard against a degenerate (side <= 0) triangle
            double ux = ex / elen, uy = ey / elen;  // unit vector along the edge
            double nx = -uy, ny = ux;               // edge normal
            if ((Mx - cx) * nx + (My - cy) * ny < 0.0) { nx = -nx; ny = -ny; } // force outward
            double angDeg = std::atan2(uy, ux) * 180.0 / PI;

            // Outer rectangle: inner side flush with the triangle edge, extending outward by H
            cv::Point2f outerCenter((float)(Mx + 0.5 * H * nx), (float)(My + 0.5 * H * ny));
            geom.outer.push_back(cv::RotatedRect(outerCenter, cv::Size2f((float)W, (float)H), (float)angDeg));

            // Inner rectangle: (fx, fy) measured in the (edge, outward) frame anchored at the outer
            // rectangle's corner on the edge-start vertex side (O = edge midpoint - W/2 along the edge).
            double Ox = Mx - 0.5 * W * ux;
            double Oy = My - 0.5 * W * uy;
            cv::Point2f innerCenter((float)(Ox + fx * ux + fy * nx), (float)(Oy + fx * uy + fy * ny));
            geom.inner.push_back(cv::RotatedRect(innerCenter, cv::Size2f((float)fw, (float)fh), (float)angDeg));
        }
        return geom;
    }

    // Filled mask (255 inside, 0 outside) for the triangle
    cv::Mat FlyTrackPlugin::triangleMask(const std::vector<cv::Point2f>& triangle) {
        cv::Mat mask = cv::Mat::zeros(bgMedianImage_.size(), CV_8U);
        cv::Point pts[3];
        for (int i = 0; i < 3; i++) pts[i] = triangle[i];
        cv::fillConvexPoly(mask, pts, 3, cv::Scalar(255));
        return mask;
    }

    // Filled mask (255 inside, 0 outside) for a rotated rectangle
    cv::Mat FlyTrackPlugin::rotatedRectMask(const cv::RotatedRect& rrect) {
        cv::Mat mask = cv::Mat::zeros(bgMedianImage_.size(), CV_8U);
        cv::Point2f v[4];
        rrect.points(v);
        cv::Point pts[4];
        for (int i = 0; i < 4; i++) pts[i] = v[i];
        cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255));
        return mask;
    }

    // Draw triangle outline
    void FlyTrackPlugin::drawTriangle(cv::Mat& img, const std::vector<cv::Point2f>& triangle, cv::Scalar color) {
        for (int i = 0; i < 3; i++) cv::line(img, triangle[i], triangle[(i + 1) % 3], color, 2);
    }

    // Draw rotated-rectangle outline
    void FlyTrackPlugin::drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rrect, cv::Scalar color) {
        cv::Point2f v[4];
        rrect.points(v);
        for (int i = 0; i < 4; i++) cv::line(img, v[i], v[(i + 1) % 4], color, 2);
    }

    // Return true if any foreground contour inside mask exceeds the fish pixel-area threshold
    bool FlyTrackPlugin::detectFishInMask(const cv::Mat& isFg, const cv::Mat& mask) {
        cv::Mat masked;
        isFg.copyTo(masked, mask);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(masked, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (size_t i = 0; i < contours.size(); i++) {
            if (cv::contourArea(contours[i]) > fish_detect_pixel_threshold) return true;
        }
        return false;
    }

    // void setROI()
    // Main ROI mask is the equilateral triangle; the outer/inner rectangles are computed on demand
    // from roiGeom_. currently only RECTANGLE roiType is implemented.
    void FlyTrackPlugin::setROI(FlyTrackConfig config) {
        roiMasksValid_ = false; // config may have changed - cached zone masks are stale
        if (!bgImageComputed_) return;
        printf("setting ROI (triangle Main ROI + 3 outer Reset + 3 inner Feeder rectangles)\n");
        switch (config.roiType) {
        case RECTANGLE:
            roiGeom_ = computeRoiGeometry(config);
            inROI_ = triangleMask(roiGeom_.triangle); // Main ROI mask = equilateral triangle
            break;
        }
    }

    //// void setBackgroundModel()
    //// set the background model fields
    //// if bgImageFilePath_ exists, load background model from file
    //// store background model in bgMedianImage_, bgLowerBoundImage_, bgUpperBoundImage_
    //// set inROI_ mask
    //void FlyTrackPlugin::setBackgroundModel() {

    //    printf("Computing background model\n");

    //    cv::Mat bgMedianImage;
    //    bool success = loadBackgroundModel(config_.bgImageFilePath, bgMedianImage);
    //    if (!success)
    //        return;

    //    // store background model
    //    storeBackgroundModel(bgMedianImage,config_);

    //    bgImageComputed_ = true;

    //}

    // void setBackgroundModel(cv::Mat& bgMedianImage)
    // set bgMedianImage_ to the input bgMedianImage
    // use background subtraction threshold to pre-compute lower bound 
    // and upper bound images, update ROI image
    // inputs:
    // bgMedianImage: median background image to store
    void FlyTrackPlugin::setBackgroundModel(cv::Mat& bgMedianImage, FlyTrackConfig& config) {

        printf("Setting background model\n");
        bgMedianImage_ = bgMedianImage.clone();
        cv::add(bgMedianImage, config_.backgroundThreshold, bgUpperBoundImage_);
        cv::subtract(bgMedianImage, config_.backgroundThreshold, bgLowerBoundImage_);
        triangleCenterXSpinBox->setRange(0, bgMedianImage.cols);
        triangleCenterYSpinBox->setRange(0, bgMedianImage.rows);
        triangleSideSpinBox->setRange(0, bgMedianImage.cols);
        triangleAngleSpinBox->setRange(0, bgMedianImage.rows);

        // roi mask
        setROI(config);

        setPreviewImage(bgMedianImage_,config);
        printf("Done\n");

        //output lower bound to file
        if (config_.DEBUG) {
            printf("Outputting background model debug images\n");
            bool success;
            QString tmpOutFile;
            tmpOutFile = config_.tmpOutDir + QString("\\bgLowerBound.png");
            printf("Writing lower bound to %s\n", tmpOutFile.toStdString().c_str());
            success = cv::imwrite(tmpOutFile.toStdString(), bgLowerBoundImage_, imwriteParams_);
            if (!success) printf("Failed writing lower bound to %s\n", tmpOutFile.toStdString().c_str());
            //output upper bound to file
            printf("Writing upper bound to %s\n", tmpOutFile.toStdString().c_str());
            tmpOutFile = config_.tmpOutDir + QString("\\bgUpperBound.png");
            success = cv::imwrite(tmpOutFile.toStdString(), bgUpperBoundImage_, imwriteParams_);
            if (!success) printf("Failed writing upper bound to %s\n", tmpOutFile.toStdString().c_str());
        }
    }

    void FlyTrackPlugin::setPreviewImage(cv::Mat matImage,FlyTrackConfig config)
    {
        if (matImage.empty()) {
			fprintf(stderr,"preview image is empty\n");
			return;
		}

        cv::Mat colorMatImage = matImage.clone();
        cv::cvtColor(colorMatImage, colorMatImage, cv::COLOR_GRAY2BGR);
        switch (config.roiType) {
            case RECTANGLE:
            {
                // Draw triangle-based ROI layout: triangle (Main, red), 3 outer (Reset, green), 3 inner (Feeder, blue).
                RoiGeometry geom = computeRoiGeometry(config);
                drawTriangle(colorMatImage, geom.triangle, cv::Scalar(0, 0, 255));
                for (int i = 0; i < 3; i++) drawRotatedRect(colorMatImage, geom.outer[i], cv::Scalar(100, 255, 0));
                for (int i = 0; i < 3; i++) drawRotatedRect(colorMatImage, geom.inner[i], cv::Scalar(255, 0, 0));
                break;
            }
        }

		QImage img = matToQImage(colorMatImage);
        if (img.isNull()) {
            fprintf(stderr,"preview image is null\n");
            return;
        }
        QPixmap pixmapOriginal = QPixmap::fromImage(img);
        QPixmap pixmapScaled = pixmapOriginal.scaled(previewImageLabel->size(), 
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation);
		previewImageLabel->setPixmap(pixmapScaled);
	}

    // void backgroundSubtraction()
    // perform background subtraction on currentImage_ and stores results in isFg_
    // use bgLowerBoundImage_, bgUpperBoundImage_ to threshold
    // difference from bgMedianImage_ to determine background/foreground membership.
    // if roiType_ is not NONE, use inROI_ mask to restrict foreground to ROI.
    // lock must be acquired outside of this function
    void FlyTrackPlugin::backgroundSubtraction() {
        // Get background/foreground membership, 255=background, 0=foreground
        switch (config_.flyVsBgMode) {
        case FLY_DARKER_THAN_BG:
            isFg_ = currentImage_ < bgLowerBoundImage_;
            break;
        case FLY_BRIGHTER_THAN_BG:
            isFg_ = currentImage_ > bgUpperBoundImage_;
            break;
        case FLY_ANY_DIFFERENCE_BG:
            cv::inRange(currentImage_, bgLowerBoundImage_, bgUpperBoundImage_, isFg_);
            cv::bitwise_not(isFg_, isFg_);
            break;
        }
        
        /*
        if (config_.roiType != NONE) {
            cv::bitwise_and(isFg_, inROI_, isFg_);
        }
        */
        

        if (config_.DEBUG && isFirst_) {
            printf("Outputting background subtraction debug images\n");
            if (!QFile::exists(config_.tmpOutDir)) {
                try {
                    QDir().mkdir(config_.tmpOutDir);
                }
                catch (std::exception& e) {
                    fprintf(stderr, "Error creating debug directory %s: %s\n", config_.tmpOutDir.toStdString().c_str(), e.what());
                }
            }
            if (QFile::exists(config_.tmpOutDir)) {
                QString tmpOutFile;
                bool success;
                cv::Mat dBkgd;
                cv::absdiff(currentImage_, bgMedianImage_, dBkgd);
                tmpOutFile = config_.tmpOutDir + QString("\\dBkgd.png");
                printf("Writing difference from background to %s\n", tmpOutFile.toStdString().c_str());
                success = cv::imwrite(tmpOutFile.toStdString(), dBkgd, imwriteParams_);
                if (!success) printf("Failed writing difference from background to %s\n", tmpOutFile.toStdString().c_str());
                tmpOutFile = config_.tmpOutDir + QString("\\isFg.png");
                printf("Writing foreground mask to %s\n", tmpOutFile.toStdString().c_str());
                success = cv::imwrite(tmpOutFile.toStdString(), isFg_);
                if (!success) printf("Failed writing foreground mask to %s\n", tmpOutFile.toStdString().c_str());
                tmpOutFile = config_.tmpOutDir + QString("\\inROI.png");
                printf("Writing ROI mask to %s\n", tmpOutFile.toStdString().c_str());
                success = cv::imwrite(tmpOutFile.toStdString(), inROI_);
                if (!success) printf("Failed writing ROI mask to %s\n", tmpOutFile.toStdString().c_str());
            }
        }
    }

    // void updateVelocityHistory()
    // update velocity history buffer velocityHistory_ and mean velocity meanFlyVelocity_ over that buffer
    // with velocity between current flyEllipse_ and previous center flyEllipseHistory_.back()
    void FlyTrackPlugin::updateVelocityHistory() {

        if (flyEllipseHistory_.size() == 0)
            return;

        double nHistory;
        // update velocity history
        cv::Point2d velocityLast;
        // compute velocity of center between current ellipse and last ellipse
        velocityLast = cv::Point2d(flyEllipse_.x - flyEllipseHistory_.back().x,
            flyEllipse_.y - flyEllipseHistory_.back().y);

        // update mean velocity for adding velocityLast
        nHistory = (double)velocityHistory_.size();
        meanFlyVelocity_ = (meanFlyVelocity_ * nHistory + velocityLast) / (nHistory + 1.0);

        // add to velocity history
        velocityHistory_.push_back(velocityLast);
        nHistory = nHistory + 1.0;

        // if we are removing from buffer, update mean velocity
        if (velocityHistory_.size() > config_.historyBufferLength) {
            meanFlyVelocity_ = (meanFlyVelocity_ * nHistory - velocityHistory_.front()) / (nHistory - 1);
            velocityHistory_.pop_front();
        }
    }

    // void updateEllipseHistory()
    // add current flyEllipse_ to end of flyEllipseHistory_
    void FlyTrackPlugin::updateEllipseHistory() {
        // add ellipse to history
        flyEllipseHistory_.push_back(flyEllipse_);
        flyEllipseDequePtr_->acquireLock();
		if (flyEllipseDequePtr_->size() >= config_.maxTrackQueueLength-1) {
			flyEllipseDequePtr_->pop_front();
		}
        flyEllipseDequePtr_->push_back(flyEllipse_);
        flyEllipseDequePtr_->releaseLock();
    }

    // void updateOrientationHistory()
    // update orientation history buffer orientationHistory_ and mean orientation meanFlyOrientation_ over that buffer
    // orientations will be stored so that they are in the same range of 2*pi
    void FlyTrackPlugin::updateOrientationHistory() {
        double nHistory;
        // update orientation history
        double currOrientation = flyEllipse_.theta;
        if (orientationHistory_.size() > 0) {
            // make orientations in same range of 2*pi
            double prevOrientation = orientationHistory_.back();
            // compute orientation change
            double orientationChange = mod2pi(currOrientation - prevOrientation);
            // this could become way out of the range -pi, pi if we run for a really long time
            currOrientation = prevOrientation + orientationChange;
        }
        // add to orientation history
        orientationHistory_.push_back(currOrientation);
        // update mean orientation for adding currOrientation
        nHistory = (double)orientationHistory_.size();
        meanFlyOrientation_ = (meanFlyOrientation_ * (nHistory - 1) + currOrientation) / nHistory;
        // if we are removing from buffer, update mean orientation
        if (orientationHistory_.size() > config_.historyBufferLength) {
            meanFlyOrientation_ = (meanFlyOrientation_ * nHistory - orientationHistory_.front()) / (nHistory - 1);
            orientationHistory_.pop_front();
        }
    }

    // void flipFlyOrientationHistory()
    // flip all orientations in orientationHistory_ and the mean meanFlyOrientation_ by adding pi
    void FlyTrackPlugin::flipFlyOrientationHistory() {
        meanFlyOrientation_ = meanFlyOrientation_ + M_PI;
        for (int i = 0; i < orientationHistory_.size(); i++) {
            orientationHistory_[i] += M_PI;
        }
    }

    // void resolveHeadTail()
    // resolve head/tail ambiguity by comparing orientation flyEllipse_.theta
    // to velocity meanFlyVelocity_ and past orientation meanFlyOrientation_
    // flyEllipse_.theta is updated 
    void FlyTrackPlugin::resolveHeadTail() {

        double velmag = 0.0;
        double dotprod;
        double costVel0 = 0.0, costVel1 = 0.0;
        double costOri0 = 0.0, costOri1 = 0.0;
        double cost0 = 0.0, cost1 = 0.0;
        double theta0 = flyEllipse_.theta;
        cv::Point2d headDir = cv::Point2d(std::cos(flyEllipse_.theta), std::sin(flyEllipse_.theta));
        cv::Point2d headDirPrev = cv::Point2d(0.0, 0.0);

        // velocity magnitude
        if (velocityHistory_.size() > 0) velmag = cv::norm(meanFlyVelocity_);

        // if fly is walking fast enough, try to match the velocity direction
        if (velmag > config_.minVelocityMagnitude) {
            dotprod = headDir.dot(meanFlyVelocity_) / velmag;
            costVel1 = dotprod;
            costVel0 = -dotprod;
            // if we haven't ever resolved headTail, we don't care about orientation history
            if (!headTailResolved_ && std::abs(dotprod) > MIN_VEL_MATCH_DOTPROD) {
                if (costVel1 < costVel0) {
                    // add pi
                    flyEllipse_.theta += M_PI;
                    flipFlyOrientationHistory();
                }
                headTailResolved_ = true;
                return;
            }
        }

        // try to match current and previous orientation
        if (orientationHistory_.size() > 0) {
            headDirPrev.x = std::cos(meanFlyOrientation_);
            headDirPrev.y = std::sin(meanFlyOrientation_);
            dotprod = headDir.dot(headDirPrev);
            costOri1 = dotprod;
            costOri0 = -dotprod;
        }

        cost0 = config_.headTailWeightVelocity * costVel0 + costOri0;
        cost1 = config_.headTailWeightVelocity * costVel1 + costOri1;
        //printf("Total cost0: %f, cost1: %f\n", cost0, cost1);

        if (cost1 < cost0) {
            // add pisiz
            flyEllipse_.theta += M_PI;
        }

        // store theta in range -pi, pi
        flyEllipse_.theta = mod2pi(flyEllipse_.theta);
    }

    void FlyTrackPlugin::logCurrentFrame(){
        if (!loggingEnabled_) return;
        if (!logFile_.isOpen()) return;
        if (!isFirst_) logStream_ << ",\n";
        logStream_ << ellipseToJson(flyEllipse_);
    }

    // helper functions

    // void loadBackgroundModel(QString bgImageFilePath, cv::Mat& bgMedianImage)
    // void loadBackgroundModel(QString bgImageFilePath, cv::Mat& bgMedianImage)
    // load background model from file with cv::imread
    // inputs:
    // bgImageFilePath: path to background image file to load
    // bgMedianImage: destination for median background image
    bool loadBackgroundModel(QString bgImageFilePath, cv::Mat& bgMedianImage) {

        if (!QFile::exists(bgImageFilePath)) {
            return false;
        }
        printf("Reading background image from %s\n", bgImageFilePath.toStdString().c_str());
        try {
            bgMedianImage = cv::imread(bgImageFilePath.toStdString(), cv::IMREAD_GRAYSCALE);
        }
		catch (cv::Exception& e) {
			fprintf(stderr, "Failed to read background image from %s: %s\n", bgImageFilePath.toStdString().c_str(), e.what());
			return false;
		}
        printf("Done\n");
        fflush(stdout);
        return true;
    }

    // OBSOLETE
    // compute the median background image from video in bgVideoFilePath_
    // inputs:
    // bgMedianImage: destination for median background image
    void computeBackgroundMedian(QString bgVideoFilePath,
        int nFramesBgEst, int lastFrameSample,
        cv::Mat& bgMedianImage,
        QProgressBar* progressBar) {
        if (bgVideoFilePath.isEmpty()) {
            fprintf(stderr, "No background video file specified\n");
            return;
        }
        else if (!QFile::exists(bgVideoFilePath)) {
            fprintf(stderr, "Background video file %s does not exist\n", bgVideoFilePath.toStdString().c_str());
            return;
        }
        videoBackend vidObj = videoBackend(bgVideoFilePath);
        int nFrames = vidObj.getNumFrames();

        StampedImage newStampedImg;
        newStampedImg.image = vidObj.grabImage();

        BackgroundData_ufmf backgroundData;
        backgroundData = BackgroundData_ufmf(newStampedImg,
            FlyTrackPlugin::BG_HIST_NUM_BINS,
            FlyTrackPlugin::BG_HIST_BIN_SIZE);
        backgroundData.addImage(newStampedImg);

        // which frames to sample
        if (nFrames < nFramesBgEst || nFramesBgEst <= 0) nFramesBgEst = nFrames;
        if (nFrames < lastFrameSample || lastFrameSample <= 0) lastFrameSample = nFrames;
        int nFramesSkip = lastFrameSample / nFramesBgEst;

        // add evenly spaced frames to the background model
        printf("Reading frames for background estimation\n");
        fflush(stdout);
        for (int f = nFramesSkip; f < lastFrameSample; f += nFramesSkip) {
            printf("Reading frame %d\n", f);
            fflush(stdout);
            vidObj.setFrame(f);
            newStampedImg.image = vidObj.grabImage();
            backgroundData.addImage(newStampedImg);
            if ((progressBar != NULL) && (progressBar->isVisible())) {
                progressBar->setValue((100.0 * (f + nFramesSkip) / lastFrameSample));
            }
        }
        printf("Finished reading.\n");
        // compute the median image
        printf("Computing median image\n");
        fflush(stdout);
        bgMedianImage = backgroundData.getMedianImage();
        printf("Done\n");
        fflush(stdout);
        backgroundData.clear();
    }




    bool FlyTrackPlugin::scanFishOutsideROI(cv::Mat& isFg, cv::Rect ROI) {
        // Summary: Mask out the ROI using 0. Then calculate the pixels that are brighter than the threshold over the entire (masked) image.
        unsigned int count_pixels_outside_ROI = 0;
        int image_width = isFg.rows;
        int image_height = isFg.cols;
        cv::Mat masked_image = isFg.clone();
        
        // Mask
        for (int row_idx = ROI.y; row_idx < ROI.y + ROI.height; row_idx++) {
            for (int col_idx = ROI.x; col_idx < ROI.x + ROI.width; col_idx++) {
                masked_image.at<uchar>(row_idx, col_idx) = 0;
            }
        }

        // Count
        cv::Mat binary_image;
        
        cv::threshold(masked_image, binary_image, fish_detect_intensity_threshold, 255, cv::THRESH_BINARY);
        unsigned int count = cv::countNonZero(binary_image);
        if (count > fish_detect_pixel_threshold) {
            return true;
        }
        

        return false;
    }

    // int largestConnectedComponent(cv::Mat& isFg)
    // find largest connected components in isFg
    // inputs:
    // isFg: binary image, 255=background, 0=foreground
    // returns: area of largest connected component
    int largestConnectedComponent(cv::Mat& isFg) {
        cv::Mat ccLabels;
        int nCCs = cv::connectedComponents(isFg, ccLabels);
        // find largest connected component
        int maxArea = 0;
        int cc = 0;
        int currArea;
        for (int i = 1; i < nCCs; i++) {
            currArea = cv::countNonZero(ccLabels == i);
            if (currArea > maxArea) {
                maxArea = currArea;
                cc = i;
            }
        }
        isFg = ccLabels == cc;
        return maxArea;
    }

    // void fitEllipse(cv::Mat& isFg, EllipseParams& flyEllipse)
    // fit an ellipse to the foreground pixels in isFg. 
    // computes the principal components of the foreground pixel locations
    // creates an ellipse with center the mean of the pixel locations,
    // orientation the angle of the first principal component,
    // semi-major and semi-minor axes twice the square roots of the eigenvalues.
    // inputs:
    // isFg: binary image, 255=background, 0=foreground
    // flyEllipse: destination for ellipse parameters

    void FlyTrackPlugin::createRotatedRectMask(cv::Mat& mask, cv::RotatedRect rotatedRect) {
        // Get the vertices of the rotated rectangle
        cv::Point2f vertices[4];
        rotatedRect.points(vertices);

        // Convert the vertices to an array of points
        std::vector<cv::Point> points;
        for (int i = 0; i < 4; i++) {
            points.push_back(vertices[i]);
        }

        // Fill the rotated rectangle on the mask
        cv::fillConvexPoly(mask, points, cv::Scalar(255));
    }

    
    //bool FlyTrackPlugin::detectOneFishInsideROI(cv::Mat& isFg, cv::Rect ROI) {
    //    cv::Mat mask = cv::Mat::zeros(isFg.size(), CV_8U);
    //    //cv::rectangle(mask, ROI, cv::Scalar(255), cv::FILLED);
    //    cv::Mat mask = rotatedRectangleROI(double centerX, double centerY, double width, double height, 5);
    //    cv::Mat masked_image;
    //    isFg.copyTo(masked_image, mask);
    //    cv::Mat binary_image;
    //    cv::threshold(masked_image, binary_image, fish_detect_intensity_threshold, 255, cv::THRESH_BINARY);

    //    unsigned int n_fish_inside_roi = 0;
    //    std::vector<std::vector<cv::Point>> contours;
    //    cv::findContours(masked_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //    for (int i = 0; i < contours.size(); i++) {
    //        cv::Moments mu = cv::moments(contours[i], false);

    //        //Area 
    //        double area = cv::contourArea(contours[i]);

    //        if (area > fish_detect_pixel_threshold) {
    //            //Calculate centroid
    //            // cv::Point2f centroid(mu.m10 / mu.m00, mu.m01 / mu.m00); Not needed right now
    //            return true;
    //        }
    //    }
    //    
    //    return false;
    //}
    
    

    bool FlyTrackPlugin::detectOneFishInsideROI_rotated(cv::Mat& isFg, double centerX, double centerY, double width, double height, double angle) {
        // This works with rotated rectangles
        cv::Mat mask = rotatedRectangleROI(centerX, centerY, width, height, angle);
        cv::Mat masked_image;
        isFg.copyTo(masked_image, mask);
        cv::Mat binary_image;
        cv::threshold(masked_image, binary_image, fish_detect_intensity_threshold, 255, cv::THRESH_BINARY);

        unsigned int n_fish_inside_roi = 0;
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(masked_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++) {
            cv::Moments mu = cv::moments(contours[i], false);

            //Area 
            double area = cv::contourArea(contours[i]);

            if (area > fish_detect_pixel_threshold) {
                //Calculate centroid
                // cv::Point2f centroid(mu.m10 / mu.m00, mu.m01 / mu.m00); Not needed right now
                return true;
            }
        }

        return false;
    }

    std::vector<cv::Point2f> FlyTrackPlugin::getFishCentroids(const cv::Mat& isFg) {
        std::vector<cv::Point2f> centroids;
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(isFg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > fish_detect_pixel_threshold) {
                cv::Moments mu = cv::moments(contour, false);
                if (mu.m00 != 0) {
                    centroids.push_back(cv::Point2f(static_cast<float>(mu.m10 / mu.m00),
                        static_cast<float>(mu.m01 / mu.m00)));
                }
            }
        }
        return centroids;
    }


    bool FlyTrackPlugin::detectOneFishInsideUnionROI_rotated(cv::Mat& isFg, double centerX, double centerY, double width, double height, double angle) {
        cv::Mat unionMask = rotatedUnionROI(centerX, centerY, width, height, angle);
        cv::Mat masked_image;        
        isFg.copyTo(masked_image, unionMask);
        cv::Mat binary_image;
        cv::threshold(masked_image, binary_image, fish_detect_intensity_threshold, 255, cv::THRESH_BINARY);

        unsigned int n_fish_inside_roi = 0;
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(masked_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++) {
            cv::Moments mu = cv::moments(contours[i], false);

            //Area 
            double area = cv::contourArea(contours[i]);

            if (area > fish_detect_pixel_threshold) {
                //Calculate centroid
                // cv::Point2f centroid(mu.m10 / mu.m00, mu.m01 / mu.m00); Not needed right now
                return true;
            }
        }
        return false;
    }



    bool FlyTrackPlugin::detectAllFishInsideROI(cv::Mat& isFg, cv::Rect ROI) {
        cv::Mat mask = cv::Mat::zeros(isFg.size(), CV_8U);
        cv::rectangle(mask, ROI, cv::Scalar(255), cv::FILLED);
        cv::Mat masked_image;
        isFg.copyTo(masked_image, mask);
        cv::Mat binary_image;
        cv::threshold(masked_image, binary_image, fish_detect_intensity_threshold, 255, cv::THRESH_BINARY);

        unsigned int n_fish_inside_roi = 0;
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(masked_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++) {
            cv::Moments mu = cv::moments(contours[i], false);

            //Area 
            double area = cv::contourArea(contours[i]);

            if (area > fish_detect_pixel_threshold) {
                //Calculate centroid
                // cv::Point2f centroid(mu.m10 / mu.m00, mu.m01 / mu.m00); Not needed right now
                n_fish_inside_roi ++;
            }
        }
       
        if (n_fish_inside_roi == 3) {
            return true;
        }
        return false;
    }



    void fitEllipse(cv::Mat& isFg, EllipseParams& flyEllipse) {

        // eigen decomposition of covariance matrix
        // this probably isn't the fastest way to do this, but
        // it seems to work
        cv::Mat fgPixels;
        cv::findNonZero(isFg, fgPixels);
		if (fgPixels.rows == 0) {
            printf("No foreground pixels found.\n");
            flyEllipse.x = 0.0;
			flyEllipse.y = 0.0;
			flyEllipse.a = 0.0;
			flyEllipse.b = 0.0;
			flyEllipse.theta = 0.0;
			return;
		}
        cv::Mat fgPixelsD = cv::Mat::zeros(fgPixels.rows, 2, CV_64F);
        for (int i = 0; i < fgPixels.rows; i++) {
            fgPixelsD.at<double>(i, 0) = fgPixels.at<cv::Point>(i).x;
            fgPixelsD.at<double>(i, 1) = fgPixels.at<cv::Point>(i).y;
        }
        cv::PCA pca_analysis(fgPixelsD, cv::Mat(), cv::PCA::DATA_AS_ROW);
        flyEllipse.x = pca_analysis.mean.at<double>(0, 0);
        flyEllipse.y = pca_analysis.mean.at<double>(0, 1);
        // orientation of ellipse (modulo pi)
        flyEllipse.theta = std::atan2(pca_analysis.eigenvectors.at<double>(0, 1),
            pca_analysis.eigenvectors.at<double>(0, 0));
        // semi major, minor axis lengths
        double lambda1 = pca_analysis.eigenvalues.at<double>(0);
        double lambda2 = pca_analysis.eigenvalues.at<double>(1);
        flyEllipse.a = std::sqrt(lambda1) * 2.0;
        flyEllipse.b = std::sqrt(lambda2) * 2.0;
    }

    double mod2pi(double angle) {
        return std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
    }

    bool checkFileExists(QString file) {
        if (file.isEmpty()) {
            return false;
        }
        return QFile::exists(file);
    }

    QString ellipseToJson(EllipseParams ell) {
        QString json = QString("{");
        json += QString("\"frame\": %1,").arg(ell.frame);
        json += QString("\"x\": %1,").arg(ell.x);
        json += QString("\"y\": %1,").arg(ell.y);
        json += QString("\"a\": %1,").arg(ell.a);
        json += QString("\"b\": %1,").arg(ell.b);
        json += QString("\"theta\": %1").arg(ell.theta);
        json += QString("}");
        return json;
    }

    QString fishStatusToJson(bool trigger) {
        QString json = QString("{");
        json += QString("\"trigger\": %1").arg(trigger);
        json += QString("}");
        return json;
    }

    // Serialize per-zone occupancy of the 3 inner (Feeder) rectangles as {"feeder": [z0, z1, z2]},
    // where each entry is 1 (fish detected in that zone) or 0. Zone index = triangle-edge order.
    QString feederStatusToJson(const std::vector<int>& zoneStatus) {
        QString json = QString("{\"feeder\": [");
        for (size_t i = 0; i < zoneStatus.size(); ++i) {
            json += QString::number(zoneStatus[i]);
            if (i + 1 < zoneStatus.size()) json += QString(", ");
        }
        json += QString("]}");
        return json;
    }

    // Serialize per-zone occupancy of the 3 outer (Reset) rectangles as {"resetROI": [z0, z1, z2]},
    // where each entry is 1 (fish detected in that zone) or 0. Zone index = triangle-edge order.
    QString resetStatusToJson(const std::vector<int>& zoneStatus) {
        QString json = QString("{\"resetROI\": [");
        for (size_t i = 0; i < zoneStatus.size(); ++i) {
            json += QString::number(zoneStatus[i]);
            if (i + 1 < zoneStatus.size()) json += QString(", ");
        }
        json += QString("]}");
        return json;
    }

    QString ROIToJson(bool trigger) {
        QString json = QString("{");
        json += QString("\"trigger\": %1").arg(trigger);
        json += QString("}");
        return json;
    }

    QString fishCentroidsToJson(const std::vector<cv::Point2f>& centroids) {
        QString json = "[";
        for (int i = 0; i < centroids.size(); ++i) {
            json += QString("{\"x\": %1, \"y\": %2}").arg(centroids[i].x).arg(centroids[i].y);
            if (i < centroids.size() - 1) {
                json += ",";
            }
        }
        json += "]";
        return json;
    }
}
