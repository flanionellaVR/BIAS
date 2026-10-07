#ifndef BIAS_IMAGE_GRABBER_HPP  
#define BIAS_IMAGE_GRABBER_HPP

#include <memory>
#include <QMutex>
#include <QObject>
#include <QRunnable>
#include <QThread>
#include "basic_types.hpp"
#include "camera_fwd.hpp"
#include "lockable.hpp"
#include "video_utils.hpp"

namespace bias
{

    struct StampedImage;

    class ImageGrabber : public QObject, public QRunnable, public Lockable<Empty>
    {
        Q_OBJECT

        public:
            ImageGrabber(QObject *parent=0);

            ImageGrabber(
                    unsigned int cameraNumber,
                    std::shared_ptr<Lockable<Camera>> cameraPtr,
                    std::shared_ptr<LockableQueue<StampedImage>> newImageQueuePtr, 
                    QObject *parent=0
                    );

            void initialize(
                    unsigned int cameraNumber,
                    std::shared_ptr<Lockable<Camera>> cameraPtr,
                    std::shared_ptr<LockableQueue<StampedImage>> newImageQueuePtr 
                    );

            void initializeVidBackend();

            void stop();
            void enableErrorCount();
            void disableErrorCount();
            void setIsVideo(bool v);
            void setVideoFileName(QString captureVideoFileName);
            void setStartFrame(int f);
            void setPlayFps(double fps); // throttle video playback to this fps (0 = flat out)

            static unsigned int DEFAULT_NUM_STARTUP_SKIP;
            static unsigned int MIN_STARTUP_SKIP;
            static unsigned int MAX_ERROR_COUNT;

        signals:
            void startTimer();
            void startCaptureError(unsigned int errorId, QString errorMsg);
            void stopCaptureError(unsigned int errorId, QString errorMsg);
            void captureError(unsigned int errorId, QString errorMsg);
            void videoFinished();

        private:
            bool ready_;
            bool stopped_;
            bool capturing_;
            bool errorCountEnabled_;
            unsigned int numStartUpSkip_;
            unsigned int cameraNumber_;

            // for reading from video instead of camera
            bool isVideo_;
            QString vidFileName_;
            int startFrame_; // video frame to start from (0 = beginning)
            double playFps_; // throttle video playback to this fps (0 = flat out, as fast as possible)
            videoBackend* vidObj_;

            std::shared_ptr<Lockable<Camera>> cameraPtr_;
            std::shared_ptr<LockableQueue<StampedImage>> newImageQueuePtr_;

            void run();
            double convertTimeStampToDouble(TimeStamp curr, TimeStamp init);
    };


    //class TSleepThread : QThread
    //{
    //    public:
    //        static void sleep(unsigned long secs) { QThread::sleep(secs); };
    //        static void msleep(unsigned long msecs) { QThread::msleep(msecs); };
    //        static void usleep(unsigned long usecs) { QThread::usleep(usecs); };
    //};


} // namespace bias


#endif // #ifndef BIAS_IMAGE_GRABBER_HPP
