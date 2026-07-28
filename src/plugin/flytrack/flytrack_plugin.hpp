#ifndef FLYTRACK_PLUGIN_HPP
#define FLYTRACK_PLUGIN_HPP
#include "ui_flytrack_plugin.h"
#include <QDialog>
#include <QWidget>
#include <QList>
#include "bias_plugin.hpp"
#include "stamped_image.hpp"
#include "background_data_ufmf.hpp"
#include "rtn_status.hpp"
#include <QDir>
#include <QTextStream>
#include <QProgressBar> // progress bar obsolete, but function is still there
#include "flytrack_config.hpp"
#include <QPointer>

class QThreadPool;

namespace cv
{
    class Mat;
}

namespace bias
{
    class CameraWindow;
    class BackgroundData_ufmf;
    class BackgroundHistogram_ufmf;
    class BackgroundMedian_ufmf;

    struct EllipseParams
    {
        int frame;
        double x;
        double y;
        double a;
        double b;
        double theta;
    };

    // Geometry for the triangle-based ROI layout:
    //   triangle : Main ROI (equilateral triangle), engulfed by
    //   outer[3] : Reset ROI rectangles, one flush on each triangle edge, extending outward
    //   inner[3] : Feeder ROI rectangles, one positioned inside each outer rectangle
    // Zone index 0-2 corresponds to triangle edge k (joining vertex k and vertex k+1).
    struct RoiGeometry
    {
        std::vector<cv::Point2f> triangle;   // 3 vertices
        std::vector<cv::RotatedRect> outer;  // 3 outer rectangles (Reset ROI)
        std::vector<cv::RotatedRect> inner;  // 3 inner rectangles (Feeder ROI)
    };

    // helper functions
    bool loadBackgroundModel(QString bgImageFilePath, cv::Mat& bgMedianImage);
    void computeBackgroundMedian(QString bgVideoFilePath, int nFramesBgEst, 
    int lastFrameSample,cv::Mat& bgMedianImage,QProgressBar* progressBar);
    int largestConnectedComponent(cv::Mat& isFg);
    void fitEllipse(cv::Mat& isFg, EllipseParams& flyEllipse);
    double mod2pi(double angle);
    QString ellipseToJson(EllipseParams ell);
    QString fishStatusToJson(bool trigger);
	QString resetStatusToJson(const std::vector<int>& zoneStatus);
    QString feederStatusToJson(const std::vector<int>& zoneStatus);
	QString fishCentroidsToJson(const std::vector<cv::Point2f>& centroids);
    bool checkFileExists(QString file);


    class FlyTrackPlugin : public BiasPlugin, public Ui::FlyTrackPluginDialog
    {
        Q_OBJECT

        public:

            static const QString PLUGIN_NAME;
            static const QString PLUGIN_DISPLAY_NAME;
            static const QString LOG_FILE_EXTENSION;
            static const QString LOG_FILE_POSTFIX;
            static const int LOGGING_PRECISION;
            static const unsigned int BG_HIST_NUM_BINS;
            static const unsigned int BG_HIST_BIN_SIZE;
            static const double MIN_VEL_MATCH_DOTPROD; // minimum dot product for velocity matching

            static const float PI;

	        static const cv::Rect ROI;
            static const cv::Rect ROI_left;
            static const cv::Rect ROI_right;
	        static const unsigned int fish_detect_intensity_threshold; // Maximum pixel intensity that defines a fish
	        static const unsigned int fish_detect_pixel_threshold; // Minimum pixel count that detects a fish outside the ROI
            static const unsigned int fish_size_threshold; // Minimum pixel count that identifies a blob as a fish
			static const float roi_angle; // Angle of the ROI that detects fish in degrees
            bool trigger_pulsed; // Flag making sure trigger is pulsed only in the first frame after a fish leaves the ROI. Refresh after all fish return to ROI
            static bool computeBackgroundModelDynamically;
            static unsigned int backgroundUpdateFramePeriod;
            bool trigger;
            bool fishInLeftFeeder;
            bool fishInRightFeeder;
            bool fishInResetROI;
            unsigned int feederStatus; // 1: left feeder, 2: right feeder, 3: both feeders, 0: no feeder
			unsigned int resetStatus; // 1: reset ROI, 0: no reset
            bool has_triggered; // Have all the fish entered the ROI since last resetting
	        //static const unsigned int ROI_x;
	        //static const unsigned int ROI_y;
	        //static const unsigned int ROI_width;
	        //static const unsigned int ROI_height;
            static const unsigned int DYNAMIC_BG_MEDIAN_UPDATE_COUNT; //frames to accumulate for bg comptute
			static const unsigned int DYNAMIC_BG_MEDIAN_UPDATE_INTERVAL; //interval between bg computes 
			static const unsigned int DYNAMIC_BG_NUM_THREADS; //number of threads for bg compute

    	FlyTrackPlugin(QWidget *parent=0);
		virtual ~FlyTrackPlugin();
            bool pluginsEnabled();
            void setPluginsEnabled(bool value);
            void getUiValues(FlyTrackConfig &config);
            void getUiBgEstValues(FlyTrackConfig& config);
            void getUiRoiValues(FlyTrackConfig& config);
            void processFramesTrackMode(QList<StampedImage> frameList);
            void processFramesBgEstMode(QList<StampedImage> frameList);
            void getCurrentImageTrackMode(cv::Mat& currentImageCopy);
            void getCurrentImageComputeBgMode(cv::Mat& currentImageCopy);
            RtnStatus popFrontTrack(EllipseParams& ell);
            RtnStatus popBackTrack(EllipseParams& ell);
            RtnStatus getLastClearTrack(EllipseParams& ell);
            RtnStatus getArenaParams(EllipseParams& ell);

            bool scanFishOutsideROI(cv::Mat& isFg, cv::Rect ROI);
			void createRotatedRectMask(cv::Mat& mask, cv::RotatedRect rotatedRect);
            void rotatedRectangle(cv::Mat& currentImageCopy, cv::Point2f center, cv::Size2f size, double rotationDegree, cv::Scalar color);
            void rectangleUnion(cv::Mat& currentImageCopy, cv::Point2f center, cv::Size2f size, double angle, cv::Scalar color);
            bool detectAllFishInsideROI(cv::Mat& isFg, cv::Rect ROI);
            //bool detectOneFishInsideROI(cv::Mat& isFg, cv::Rect ROI);
            bool detectOneFishInsideROI_rotated(cv::Mat& isFg, double centerX, double centerY, double width, double height, double angle);
            bool detectOneFishInsideUnionROI_rotated(cv::Mat& isFg, double centerX, double centerY, double width, double height, double angle);
            std::vector<cv::Point2f>getFishCentroids(const cv::Mat& isFg);

            // Triangle-based ROI geometry (see RoiGeometry). Single source of truth shared by
            // detection, mask building, and preview drawing.
            RoiGeometry computeRoiGeometry(const FlyTrackConfig& config);
            cv::Mat triangleMask(const std::vector<cv::Point2f>& triangle);
            cv::Mat rotatedRectMask(const cv::RotatedRect& rrect);
            void drawTriangle(cv::Mat& img, const std::vector<cv::Point2f>& triangle, cv::Scalar color);
            void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rrect, cv::Scalar color);
            bool detectFishInMask(const cv::Mat& isFg, const cv::Mat& mask);


            QPointer<CameraWindow> getCameraWindow();

            virtual void reset();
            virtual void stop();
            virtual void setActive(bool value);
            virtual void processFrames(QList<StampedImage> frameList);
            virtual void setFileAutoNamingString(QString autoNamingString);
            virtual void setFileVersionNumber(unsigned verNum);
            virtual cv::Mat getCurrentImage();
            virtual QString getName();
            virtual QString getDisplayName();
            virtual QVariantMap getConfigAsMap();  
            RtnStatus setFromConfig(FlyTrackConfig config);
            virtual RtnStatus setConfigFromMap(QVariantMap configMap);
            virtual RtnStatus setConfigFromJson(QByteArray jsonArray);
            virtual RtnStatus runCmdFromMap(QVariantMap cmdMap, bool showErrorDlg=true, QString& value = QString(""));
            virtual QString getLogFileExtension();
            virtual QString getLogFilePostfix();
            virtual QString getLogFileName(bool includeAutoNaming);
            virtual QString getLogFileFullPath(bool includeAutoNaming);
            virtual void showEvent(QShowEvent *event);

        
        signals:

            void setCaptureDurationRequest(unsigned long);

        protected:

            QWidget *parent_;

            void initialize();
            void initializeUi();
            void setUiEnabled();
            void setRoiUIValues();
            void connectWidgets();
            bool setBgImageFilePath(QString newBgImageFilePath);
            bool saveBgMedianImage(cv::Mat bgMedianImage, QString bgImageFilePath);
            void setPreviewImage(cv::Mat matImage, FlyTrackConfig config);

            //void setBackgroundModel();
            void setBackgroundModel(cv::Mat& bgMedianImage, FlyTrackConfig& config);
            cv::Mat circleROI(double centerX, double centerY, double centerRadius);
            cv::Mat rectangleROI(double centerX, double centerY, double width, double height);
            cv::Mat rotatedRectangleROI(double centerX, double centerY, double width, double height, double angle);
            cv::Mat rotatedUnionROI(double centerX, double centerY, double width, double height, double angle);
            void backgroundSubtraction();
            void setROI(FlyTrackConfig config);
            void updateVelocityHistory();
            void updateOrientationHistory();
            void updateEllipseHistory();
            void resolveHeadTail();
            void flipFlyOrientationHistory();
            void logCurrentFrame();
            void finishComputeBgMode();

            bool active_;
            bool requireTimer_;
            cv::Mat currentImage_;

            double timeStamp_;
            unsigned long frameCount_;

            QString fileAutoNamingString_;
            unsigned int fileVersionNumber_;

            QDir logFileDir_;
            bool loggingEnabled_;
            QFile logFile_;
            QTextStream logStream_;

            // parameters
            FlyTrackConfig config_; 

            // background model
            cv::Mat bgMedianImage_; // median background image
            cv::Mat bgLowerBoundImage_; // lower bound image for background
            cv::Mat bgUpperBoundImage_; // upper bound image for background
			bool bgImageComputed_; // flag indicating if background image has been computed
            void startDynamicBackgroundModeling();
			void stopDynamicBackgroundModeling();

            QPointer<QThreadPool> bgThreadPoolPtr_; // thread pool for background reading
			QPointer<BackgroundHistogram_ufmf> bgHistogramPtr_; // background histogram object
			QPointer<BackgroundMedian_ufmf> bgMedianPtr_; // background median object

			std::shared_ptr<LockableQueue<StampedImage>> bgImageQueuePtr_; // camera frames are pushed from main thread here. Pulled out by bg histogram thread
			std::shared_ptr<LockableQueue<BackgroundData_ufmf>> bgNewDataQueuePtr_; // new background data pushed here by bg histogram thread. Pulled out by bg median thread
			std::shared_ptr<LockableQueue<BackgroundData_ufmf>> bgOldDataQueuePtr_; // old background data pushed here by bg histogram thread. Pulled out by bg median thread
			std::shared_ptr<LockableQueue<cv::Mat>> bgMedianMatQueuePtr_; // median images pushed here by bg median thread. Pulled out by main thread

            bool dynamicBgModelingStarted_;
            BackgroundData_ufmf backgroundData_; // background estimation data
            int lastFrameAdded_; // last frame added to background model
            int nFramesAddedBgEst_; // number of frames added to background model so far

			// processing of current frame
            bool isFirst_; // flag indicating if this is the first frame
            cv::Mat isFg_; // foreground mask
            cv::Mat inROI_; // mask for ROI
            cv::Mat inROI_left_; //mask for left ROI
            cv::Mat inROI_right_; //mask for right ROI
			cv::Mat inROI_reset_; //mask for reset ROI
            EllipseParams flyEllipse_; // fly ellipse parameters
            int lastFramePreviewed_; // last frame shown in preview window
            int lastFrameMedianComputed_; // last frame median computed
            cv::Mat lastImagePreviewed_; // last image shown in preview window

            // tracking history
            std::shared_ptr<LockableDeque<EllipseParams>> flyEllipseDequePtr_; // queue for serving tracking
            std::vector<EllipseParams> flyEllipseHistory_; // tracked ellipses
            std::deque<cv::Point2d> velocityHistory_; // tracked velocity buffer
            std::deque<double> orientationHistory_; // tracked orientation buffer
            cv::Point2d meanFlyVelocity_; // mean velocity of fly
            double meanFlyOrientation_; // mean orientation of fly
            bool headTailResolved_; // flag indicating if head-tail orientation has been resolved ever

            // Fish centroids
			std::vector<cv::Point2f> fishCentroids_; // vector of fish centroids detected in the current frame

            // Triangle-based ROI geometry and per-zone fish occupancy (index 0-2 = triangle edge order)
            RoiGeometry roiGeom_;
            std::vector<int> outerZoneStatus_; // per outer (Reset) rectangle: 1 if fish inside, else 0
            std::vector<int> innerZoneStatus_; // per inner (Feeder) rectangle: 1 if fish inside, else 0

            // for writing images
            std::vector<int> imwriteParams_;

            void setRequireTimer(bool value);
            void openLogFile();
            void closeLogFile();

        private slots:

            void applyPushButtonClicked();
            void donePushButtonClicked();
            void cancelPushButtonClicked();

            void loadBgPushButtonClicked();
            void roiUiChanged(int v);
            void bgImageFilePathToolButtonClicked();
            void logFilePathToolButtonClicked();
            void tmpOutDirToolButtonClicked();
            void computeBgModeComboBoxChanged();

    };

}


#endif 


