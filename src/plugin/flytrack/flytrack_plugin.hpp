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

namespace cv
{
    class Mat;
}

namespace bias
{
    class CameraWindow;

    struct EllipseParams
    {
        int frame = 0;
        double x = 0.0;
        double y = 0.0;
        double a = 0.0;
        double b = 0.0;
        double theta = 0.0;
        double captureTime = 0.0;    // capture time of the frame (s since capture start, StampedImage.timeStamp)
        // wing tracking (valid only when wing tracking is enabled / nWingsDetected > 0)
        double wingAngleL = 0.0;     // left wing angle (rad), relative to theta+pi
        double wingAngleR = 0.0;     // right wing angle (rad), relative to theta+pi
        double wingTroughAngle = 0.0;// angle of the trough between the wings (rad)
        int nWingsDetected = 0;      // 0, 1, or 2
        double wingAreaL = 0.0;      // left wing pixel area
        double wingAreaR = 0.0;      // right wing pixel area
        // head/tail diagnostics (keep = current theta, flip = theta+pi). The cost terms
        // are UNWEIGHTED; final decision was: flip if (wVel*vel + wWing*wing + wOri*ori)
        // is smaller for flip than keep. Lets the weighting be re-derived offline.
        double htScoreKeep = 0.0, htScoreFlip = 0.0;  // angle-weighted wing-pixel score in rear window
        double htVelKeep = 0.0, htVelFlip = 0.0;      // velocity cost (unweighted)
        double htOriKeep = 0.0, htOriFlip = 0.0;      // orientation-history cost (unweighted)
        double htWingKeep = 0.0, htWingFlip = 0.0;    // wing cost (unweighted, = -normalized score)
        double htCostKeep = 0.0, htCostFlip = 0.0;    // final weighted cost actually compared
    };

    // Result of fitting wings for one head-orientation hypothesis
    struct WingFitResult
    {
        double angleL;      // smaller (left) wing angle (rad), relative to headTheta+pi
        double angleR;      // larger (right) wing angle (rad)
        double troughAngle; // angle of the trough between wings (rad)
        int nWings;         // 0, 1, or 2
        double areaL;       // left wing pixel area
        double areaR;       // right wing pixel area
        double score;       // angle-weighted wing-pixel score in the rear window: sum of
                            // cos(d)-cos(maxAngle) over retained pixels (head/tail discriminator,
                            // upweights pixels directly behind the head, 0 at the +/-maxAngle edge)
        int nRearPx;        // count of wing pixels retained in the rear window (unweighted; used
                            // for the head/tail magnitude gate so minSingleWingArea stays a pixel count)
    };

    // helper functions
    bool loadBackgroundModel(QString bgImageFilePath, cv::Mat& bgMedianImage);
    void computeBackgroundMedian(QString bgVideoFilePath, int nFramesBgEst, 
    int lastFrameSample,cv::Mat& bgMedianImage,QProgressBar* progressBar);
    int largestConnectedComponent(cv::Mat& isFg);
    void fitEllipse(cv::Mat& isFg, EllipseParams& flyEllipse);
    double mod2pi(double angle);
    QString ellipseToJson(EllipseParams ell);
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

            FlyTrackPlugin(QWidget *parent=0);
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
            void setTrajectoryFileName(QString path); // override output trajectory path (e.g. from CLI)
            void setDebugSegAllFrames(bool value); // CLI: dump wing segmentation every frame (default: first frame only)
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
            void backgroundSubtraction();
            void setROI(FlyTrackConfig config);
            void updateVelocityHistory();
            void updateOrientationHistory();
            void updateEllipseHistory();
            void resolveHeadTail(double wingScoreKeep, double wingScoreFlip, int wingRearPxKeep, int wingRearPxFlip, bool wingValid);
            void flipFlyOrientationHistory();
            // wing tracking
            void getUiWingValues(FlyTrackConfig& config);
            void trackWings(); // segment + fit both hypotheses, resolve head/tail, store into flyEllipse_
            // positive-on-fly background difference (per flyVsBgMode, ROI-masked) over a box
            void computeBackgroundDiff(const cv::Rect& box, cv::Mat& dBkgdOut);
            void segmentWingPixels(std::vector<cv::Point>& wingPx); // orientation-independent (computes diff on a box around the fly)
            static WingFitResult fitWingsFromPixels(const std::vector<cv::Point>& wingPx,
                double x, double y, double headTheta, const FlyTrackConfig& config); // orientation-dependent
            void logCurrentFrame();
            void finishComputeBgMode();

            int clampToInt(double value);

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

            BackgroundData_ufmf backgroundData_; // background estimation data
            int lastFrameAdded_; // last frame added to background model
            int nFramesAddedBgEst_; // number of frames added to background model so far

			// processing of current frame
            bool isFirst_; // flag indicating if this is the first frame
            cv::Mat isFg_; // foreground mask
            cv::Mat dBkgd_; // positive-on-fly background difference (reused for wing tracking)
            cv::Mat inROI_; // mask for ROI
            std::vector<cv::Point> wingPx_; // reusable buffer of wing-pixel coordinates
            bool debugSegAllFrames_ = false; // CLI: dump wing segmentation every frame, not just the first
            cv::Rect wingSegBox_; // box of the last wing segmentation (for the preview overlay)
            cv::Mat wingSegLabels_; // box-sized CV_8U seg labels: 1=body, 2=wing (for the preview overlay)
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


