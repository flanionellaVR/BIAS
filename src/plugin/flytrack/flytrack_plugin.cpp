#include "flytrack_plugin.hpp"
#include <QtDebug>
#include <QMessageBox>
#include <QFileDialog>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "camera_window.hpp"
#include <iostream>
#include <algorithm>
#include <vector>
#include "video_utils.hpp"
#define _USE_MATH_DEFINES
#include <math.h>
#include "mat_to_qimage.hpp"

namespace bias
{

    const QString FlyTrackPlugin::PLUGIN_NAME = QString("FlyTrack"); 
    const QString FlyTrackPlugin::PLUGIN_DISPLAY_NAME = QString("Fly Track");
    const QString FlyTrackPlugin::LOG_FILE_EXTENSION = QString("json");
    const QString FlyTrackPlugin::LOG_FILE_POSTFIX = QString("flytrack");
    const int FlyTrackPlugin::LOGGING_PRECISION = 6;

    const unsigned int FlyTrackPlugin::BG_HIST_NUM_BINS = 256;
    const unsigned int FlyTrackPlugin::BG_HIST_BIN_SIZE = 1;
    const double FlyTrackPlugin::MIN_VEL_MATCH_DOTPROD = 0.25;

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
        //config_.roiCenterX = 468.6963;
        //config_.roiCenterY = 480.2917;
        //config_.roiRadius = 428.3618;

        imwriteParams_.push_back(cv::IMWRITE_PNG_COMPRESSION);
        imwriteParams_.push_back(0);

        // parameters for resolving head/tail ambiguity
        //historyBufferLength_ = 5;
        //minVelocityMagnitude_ = 1.0; // .05; // could do this in pixels / second since we have timestamps
        //headTailWeightVelocity_ = 3.0; // weight of head-tail dot product vs previous orientation dot product

        bgImageComputed_ = false;
        active_ = false;
        lastFramePreviewed_ = -1;
        flyEllipseDequePtr_ = std::make_shared<LockableDeque<EllipseParams>>();
        initialize();


        setRequireTimer(false);
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
        StampedImage latestFrame = frameList.back();
        frameList.clear();
        currentImage_ = latestFrame.image;
        timeStamp_ = latestFrame.timeStamp;
        frameCount_ = latestFrame.frameCount;

        if (!bgImageComputed_) {
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
        // mismatched sizes
        if ((bgMedianImage_.rows != currentImage_.rows) || (bgMedianImage_.cols != currentImage_.cols)
            || bgMedianImage_.type() != currentImage_.type())
        {
            fprintf(stderr, "Background model and current image are not the same size\n");
			return;
		}

        // Get background/foreground membership, 255=background, 0=foreground
        backgroundSubtraction();

        // find connected components in isFg_
        int ccArea = largestConnectedComponent(isFg_);

        // compute mean and covariance of pixels in foreground
        flyEllipse_.frame = frameCount_;
        flyEllipse_.captureTime = timeStamp_;
        fitEllipse(isFg_, flyEllipse_);

        // default wing fields (overwritten by trackWings() when wing tracking is enabled)
        flyEllipse_.wingAngleL = 0.0;
        flyEllipse_.wingAngleR = 0.0;
        flyEllipse_.wingTroughAngle = 0.0;
        flyEllipse_.nWingsDetected = 0;
        flyEllipse_.wingAreaL = 0.0;
        flyEllipse_.wingAreaR = 0.0;

        // store velocity
        updateVelocityHistory();

        // wing tracking (if enabled) runs before head/tail and feeds it a fit score;
        // trackWings() segments wings, fits both orientation hypotheses, resolves head/tail,
        // and stores the chosen wing fit into flyEllipse_.
        if (config_.trackWings) {
            trackWings();
        }
        else {
            resolveHeadTail(0.0, 0.0, 0, 0, false);
        }

        // store ellipse
        updateEllipseHistory();

        // store orientation
        updateOrientationHistory();

        if (loggingEnabled_) {
            logCurrentFrame();
        }

        isFirst_ = false;

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
        
        if (latestFrame.frameCount <= static_cast<unsigned long>(lastFrameAdded_ + config_.nFramesSkipBgEst)) {
			return;
		}
        backgroundData_.addImage(latestFrame);
        lastFrameAdded_ += config_.nFramesSkipBgEst;
        nFramesAddedBgEst_ += 1;

    }

    // Half-box around the fly = ceil(WING_BBOX_A_FACTOR * a) + morphology margin. Bounds both
    // the wing segmentation and the preview zoom, so both scale with the fly's apparent size
    // (and adapt if the video is zoomed in/out -- no hard-coded pixel sizes). Raise if wings
    // get clipped.
    static const double WING_BBOX_A_FACTOR = 5.0;

    void FlyTrackPlugin::getCurrentImageTrackMode(cv::Mat& currentImageCopy)
    {
        if (!bgImageComputed_) {
            currentImageCopy = currentImage_.clone();
            return;
		}

        // matplotlib C0 (blue) / C1 (orange), BGR -- shared with make_overlay_video.py
        const cv::Scalar COLOR_BODY(180, 119, 31);
        const cv::Scalar COLOR_WING(14, 127, 255);

        // Background: real image for the wing-seg view, binary foreground for the normal view.
        const cv::Mat& srcGray = config_.showWingSegmentation ? currentImage_ : isFg_;
        int W = srcGray.cols, H = srcGray.rows;
        if (W == 0 || H == 0) { currentImageCopy = currentImage_.clone(); return; }

        // Crop-first: when zooming, convert and draw only the fly-sized box (then upscale to the
        // display size), instead of rendering the whole frame and cropping. cropBox = full frame
        // when not zooming. All overlays are drawn offset by the crop origin via P().
        cv::Rect cropBox(0, 0, W, H);
        bool zoom = config_.zoomToFly && flyEllipse_.a > 0.0;
        if (zoom) {
            int margin = std::max(1, config_.radiusDilateBody) + std::max(1, config_.radiusOpenWing) + 2;
            int halfH = (int)std::ceil(WING_BBOX_A_FACTOR * flyEllipse_.a) + margin;
            int halfW = std::max(1, (int)std::lround(halfH * (double)W / (double)H)); // keep aspect
            int bw = std::min(2 * halfW, W), bh = std::min(2 * halfH, H);
            int x0 = std::min(std::max(0, clampToInt(flyEllipse_.x) - bw / 2), W - bw);
            int y0 = std::min(std::max(0, clampToInt(flyEllipse_.y) - bh / 2), H - bh);
            cropBox = cv::Rect(x0, y0, bw, bh);
        }
        const int ox = cropBox.x, oy = cropBox.y;
        auto P = [&](double px, double py) { return cv::Point(clampToInt(px) - ox, clampToInt(py) - oy); };

        // base image: convert only the crop box to BGR (== whole frame when not zooming)
        cv::cvtColor(srcGray(cropBox), currentImageCopy, cv::COLOR_GRAY2BGR);

        if (config_.showWingSegmentation) {
            // transparent body/wing overlay over the part of the wing-seg box inside the crop
            cv::Rect segBox = wingSegBox_;
            if (segBox.width > 0 && segBox.height > 0 && wingSegLabels_.size() == segBox.size()) {
                cv::Rect vis = segBox & cropBox;
                if (vis.width > 0 && vis.height > 0) {
                    const double alpha = 0.45; // overlay opacity (fly visible underneath)
                    cv::Vec3b body((uchar)COLOR_BODY[0], (uchar)COLOR_BODY[1], (uchar)COLOR_BODY[2]);
                    cv::Vec3b wing((uchar)COLOR_WING[0], (uchar)COLOR_WING[1], (uchar)COLOR_WING[2]);
                    cv::Mat roi = currentImageCopy(cv::Rect(vis.x - ox, vis.y - oy, vis.width, vis.height));
                    for (int yy = 0; yy < roi.rows; yy++) {
                        const unsigned char* lp =
                            wingSegLabels_.ptr<unsigned char>(vis.y - segBox.y + yy) + (vis.x - segBox.x);
                        cv::Vec3b* rp = roi.ptr<cv::Vec3b>(yy);
                        for (int xx = 0; xx < roi.cols; xx++) {
                            if (lp[xx] == 0) continue;
                            const cv::Vec3b& c = (lp[xx] == 2) ? wing : body;
                            for (int ch = 0; ch < 3; ch++)
                                rp[xx][ch] = (uchar)(alpha * c[ch] + (1.0 - alpha) * rp[xx][ch]);
                        }
                    }
                }
            }
            // wing-fit lines + centroid (no ellipse)
            if (config_.trackWings && flyEllipse_.nWingsDetected > 0) {
                double wingLen = 2.0 * flyEllipse_.a, rear = flyEllipse_.theta + M_PI;
                double wa[2] = { flyEllipse_.wingAngleL, flyEllipse_.wingAngleR };
                cv::Point center = P(flyEllipse_.x, flyEllipse_.y);
                for (int w = 0; w < 2; w++) {
                    if (flyEllipse_.nWingsDetected < 2 && std::abs(wa[w]) < 1e-6) continue;
                    double ang = rear + wa[w];
                    cv::line(currentImageCopy, center,
                             P(flyEllipse_.x + wingLen * std::cos(ang), flyEllipse_.y + wingLen * std::sin(ang)),
                             COLOR_WING, 1, cv::LINE_AA);
                }
                cv::drawMarker(currentImageCopy, center, cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 6, 1);
            }
        }
        else {
            cv::ellipse(currentImageCopy, P(flyEllipse_.x, flyEllipse_.y),
                    cv::Size(clampToInt(flyEllipse_.a), clampToInt(flyEllipse_.b)),
                    flyEllipse_.theta * 180.0 / M_PI, 0, 360, cv::Scalar(0, 0, 255), 2);
            cv::drawMarker(currentImageCopy,
                    P(flyEllipse_.x + flyEllipse_.a * std::cos(flyEllipse_.theta),
                      flyEllipse_.y + flyEllipse_.a * std::sin(flyEllipse_.theta)),
                    cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 10, 2);
            // plot wings (lines from the body centroid toward each wing tip, behind the body)
            if (config_.trackWings && flyEllipse_.nWingsDetected > 0) {
                double wingLen = 2.0 * flyEllipse_.a, rear = flyEllipse_.theta + M_PI;
                double wa[2] = { flyEllipse_.wingAngleL, flyEllipse_.wingAngleR };
                cv::Point center = P(flyEllipse_.x, flyEllipse_.y);
                for (int w = 0; w < 2; w++) {
                    // skip the padded phantom (zero-angle) wing when only one wing is detected
                    if (flyEllipse_.nWingsDetected < 2 && std::abs(wa[w]) < 1e-6) continue;
                    double ang = rear + wa[w];
                    cv::line(currentImageCopy, center,
                             P(flyEllipse_.x + wingLen * std::cos(ang), flyEllipse_.y + wingLen * std::sin(ang)),
                             cv::Scalar(0, 255, 0), 1);
                }
            }
        }

        // When zooming we return the small cropped box as-is; the preview widget scales it up
        // to the display size (KeepAspectRatio), so the QImage/pixmap/scale all stay box-sized
        // instead of full-frame. (The box already has the frame's aspect ratio, so the framing
        // is unchanged; the widget's smooth scaling replaces the previous crisp NEAREST upscale.)
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
        // Wait at most 1 ms for the tracking lock: enough to smooth the preview when the tracker
        // is briefly between frames, but small enough that the GUI thread (which also serves the
        // HTTP server) is never stalled noticeably. If the tracker still holds the lock, reuse
        // the last rendered frame and return -- a dropped preview frame is fine.
        // (lastImagePreviewed_ is only ever written here, on the GUI thread, so reading it
        // lock-free is safe.)
        const int PREVIEW_LOCK_TRY_MS = 1;
        if (!tryLock(PREVIEW_LOCK_TRY_MS)) {
            return lastImagePreviewed_;
        }
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
        if (config_.roiType == NONE) return rtnStatus;
		ell.x = config_.roiCenterX;
		ell.y = config_.roiCenterY;
		ell.a = config_.roiRadius;
		ell.b = config_.roiRadius;
		ell.theta = 0.0;
        return rtnStatus;
    }

    QVariantMap FlyTrackPlugin::getConfigAsMap()  
    {
        QVariantMap configMap = config_.toMap();
        return configMap;
    }

    void FlyTrackPlugin::setRoiUIValues() {
        roiTypeComboBox->setCurrentIndex(config_.roiType);
        roiCenterXSpinBox->setValue(config_.roiCenterX);
        roiCenterYSpinBox->setValue(config_.roiCenterY);
        roiRadiusSpinBox->setValue(config_.roiRadius);

        if(config_.roiType == NONE) {
            roiCenterXSpinBox->setEnabled(false);
			roiCenterYSpinBox->setEnabled(false);
			roiRadiusSpinBox->setEnabled(false);
		} else {
            roiCenterXSpinBox->setEnabled(true);
			roiCenterYSpinBox->setEnabled(true);
			roiRadiusSpinBox->setEnabled(true);
		}

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
            roiCenterXSpinBox,
            SIGNAL(valueChanged(int)),
            this,
            SLOT(roiUiChanged(int))
        );
        connect(
            roiCenterYSpinBox,
            SIGNAL(valueChanged(int)),
            this,
            SLOT(roiUiChanged(int))
        );
        connect(
            roiRadiusSpinBox,
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

            // Mutate shared tracking state (config_, bgMedianImage_, inROI_) under the plugin
            // lock: this runs on the GUI thread while the tracking thread reads config_ every
            // frame (e.g. config_.wingFracFilter in fitWingsFromPixels). Without the lock, the
            // wholesale config_ reassignment frees that vector mid-read -> crash. processFrames
            // and getCurrentImage take the same lock, so this serializes against both.
            acquireLock();
            config_ = config;
            setBgImageFilePath(config_.bgImageFilePath);
            setROI(config);
            releaseLock();

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
            historyBufferLengthSpinBox->setValue(config_.historyBufferLength);
            minVelocityMagnitudeLineEdit->setText(QString::number(config_.minVelocityMagnitude));
            headTailWeightVelocityLineEdit->setText(QString::number(config_.headTailWeightVelocity));
            headTailWeightWingLineEdit->setText(QString::number(config_.headTailWeightWing));
            // wing tracking widgets
            trackWingsCheckBox->setChecked(config_.trackWings);
            normalizeWingByBackgroundCheckBox->setChecked(config_.normalizeWingByBackground);
            showWingSegmentationCheckBox->setChecked(config_.showWingSegmentation);
            zoomToFlyCheckBox->setChecked(config_.zoomToFly);
            mindWingHighSpinBox->setValue(config_.mindWingHigh);
            mindWingLowSpinBox->setValue(config_.mindWingLow);
            mindBodySpinBox->setValue(config_.mindBody);
            maxWingPxAngleLineEdit->setText(QString::number(config_.maxWingPxAngleDeg));
            minNonzeroWingAngleLineEdit->setText(QString::number(config_.minNonzeroWingAngleDeg));
            wingMinPeakDistBinsSpinBox->setValue(config_.wingMinPeakDistBins);
            wingMinPeakThresholdFracLineEdit->setText(QString::number(config_.wingMinPeakThresholdFrac));
            nBinsDThetaWingSpinBox->setValue(config_.nBinsDThetaWing);
            wingPeakMinFracFactorLineEdit->setText(QString::number(config_.wingPeakMinFracFactor));
            minSingleWingAreaSpinBox->setValue(config_.minSingleWingArea);
            radiusDilateBodySpinBox->setValue(config_.radiusDilateBody);
            radiusOpenWingSpinBox->setValue(config_.radiusOpenWing);
            wingRadiusQuadfitBinsSpinBox->setValue(config_.wingRadiusQuadfitBins);
            {
                QStringList wff;
                for (size_t i = 0; i < config_.wingFracFilter.size(); i++)
                    wff << QString::number(config_.wingFracFilter[i]);
                wingFracFilterLineEdit->setText(wff.join(","));
            }
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

    // Set the output trajectory file path (e.g. from the command line). Sets the absolute
    // path so the trajectory is written there (and openLogFile triggers without Logging).
    void FlyTrackPlugin::setTrajectoryFileName(QString path) {
        config_.tmpTrackFilePath = path;
        logFilePathLineEdit->setText(path);
    }

    void FlyTrackPlugin::setDebugSegAllFrames(bool value) {
        debugSegAllFrames_ = value;
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
        double roiCenterX = roiCenterXSpinBox->value();
        double roiCenterY = roiCenterYSpinBox->value();
        double roiRadius = roiRadiusSpinBox->value();
        config.setRoiParams(roiType, roiCenterX, roiCenterY, roiRadius);
    }

    void FlyTrackPlugin::getUiBgEstValues(FlyTrackConfig& config) {
        config.computeBgMode = computeBgModeComboBox->currentIndex() == 0;
        config.bgImageFilePath = bgImageFilePathLineEdit->text();
        config.nFramesSkipBgEst = nFramesSkipLineEdit->text().toInt();
    }

    void FlyTrackPlugin::getUiWingValues(FlyTrackConfig& config) {
        config.trackWings = trackWingsCheckBox->isChecked();
        config.normalizeWingByBackground = normalizeWingByBackgroundCheckBox->isChecked();
        config.showWingSegmentation = showWingSegmentationCheckBox->isChecked();
        config.zoomToFly = zoomToFlyCheckBox->isChecked();
        config.mindWingHigh = mindWingHighSpinBox->value();
        config.mindWingLow = mindWingLowSpinBox->value();
        config.mindBody = mindBodySpinBox->value();
        config.maxWingPxAngleDeg = maxWingPxAngleLineEdit->text().toDouble();
        config.minNonzeroWingAngleDeg = minNonzeroWingAngleLineEdit->text().toDouble();
        config.wingMinPeakDistBins = wingMinPeakDistBinsSpinBox->value();
        config.wingMinPeakThresholdFrac = wingMinPeakThresholdFracLineEdit->text().toDouble();
        config.nBinsDThetaWing = nBinsDThetaWingSpinBox->value();
        config.wingPeakMinFracFactor = wingPeakMinFracFactorLineEdit->text().toDouble();
        config.minSingleWingArea = minSingleWingAreaSpinBox->value();
        config.radiusDilateBody = radiusDilateBodySpinBox->value();
        config.radiusOpenWing = radiusOpenWingSpinBox->value();
        config.wingRadiusQuadfitBins = wingRadiusQuadfitBinsSpinBox->value();
        // parse comma-separated smoothing filter
        QStringList parts = wingFracFilterLineEdit->text().split(",", Qt::SkipEmptyParts);
        std::vector<double> filt;
        for (int i = 0; i < parts.size(); i++) {
            bool ok = false;
            double v = parts[i].trimmed().toDouble(&ok);
            if (ok) filt.push_back(v);
        }
        if (!filt.empty()) config.wingFracFilter = filt;
    }

    void FlyTrackPlugin::getUiValues(FlyTrackConfig& config) {
        try {
            getUiBgEstValues(config);
            config.flyVsBgMode = (FlyVsBgModeType)flyVsBgModeComboBox->currentIndex();
            config.backgroundThreshold = backgroundThresholdLineEdit->text().toInt();
            getUiRoiValues(config);
            config.historyBufferLength = historyBufferLengthSpinBox->value();
            config.minVelocityMagnitude = minVelocityMagnitudeLineEdit->text().toDouble();
            config.headTailWeightVelocity = headTailWeightVelocityLineEdit->text().toDouble();
            config.headTailWeightWing = headTailWeightWingLineEdit->text().toDouble();
            config.tmpOutDir = tmpOutDirLineEdit->text();
            config.DEBUG = DEBUGCheckBox->isChecked();
            config.tmpTrackFilePath = logFilePathLineEdit->text();
            config.trackFileName = logFileNameLineEdit->text();
            getUiWingValues(config);
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
        historyBufferLengthSpinBox->setEnabled(!v);
        historyBufferLengthLabel->setEnabled(!v);
        minVelocityMagnitudeLineEdit->setEnabled(!v);
        minVelocityMagnitudeLabel->setEnabled(!v);
        headTailWeightVelocityLineEdit->setEnabled(!v);
        headTailWeightVelocityLabel->setEnabled(!v);
        logFilePathLineEdit->setEnabled(!v);
        logFilePathLabel->setEnabled(!v);

        roiTypeComboBox->setEnabled(!v);
        roiTypeLabel->setEnabled(!v);
        switch (config.roiType) {
            case NONE:
                roiCenterXSpinBox->setEnabled(false);
                roiCenterXLabel->setEnabled(false);
                roiCenterYSpinBox->setEnabled(false);
                roiCenterYLabel->setEnabled(false);
                roiRadiusSpinBox->setEnabled(false);
                roiRadiusLabel->setEnabled(false);
                break;
            case CIRCLE:
                roiCenterXSpinBox->setEnabled(!v);
                roiCenterXLabel->setEnabled(!v);
                roiCenterYSpinBox->setEnabled(!v);
                roiCenterYLabel->setEnabled(!v);
                roiRadiusSpinBox->setEnabled(!v);
                roiRadiusLabel->setEnabled(!v);
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
        // Write the trajectory file if global Logging is on OR an Output Trajectory File
        // Name / Absolute Path has been set in the FlyTrack config. The latter lets you
        // output tracks without enabling Logging (so no video is recorded) -- e.g. when
        // debugging from a video file.
        loggingEnabled_ = getCameraWindow() -> isLoggingEnabled()
                          || config_.trackFileNameSet() || config_.trackFilePathSet();
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
        cv::circle(mask, cv::Point(clampToInt(centerX), clampToInt(centerY)), clampToInt(centerRadius), cv::Scalar(255), -1);
        return mask;
    }

    // void setROI()
    // set the region of interest mask based on roiType_
    // currently only circle implemented
    void FlyTrackPlugin::setROI(FlyTrackConfig config) {
        if (!bgImageComputed_) return;
        printf("setting ROI\n");
        // roi mask
        switch (config.roiType) {
        case CIRCLE:
            printf("setting circle ROI: center %f, %f, radius %f\n", config.roiCenterX, config.roiCenterY, config.roiRadius);
            inROI_ = circleROI(config.roiCenterX, config.roiCenterY, config.roiRadius);
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
        roiCenterXSpinBox->setRange(0, bgMedianImage.cols);
        roiCenterYSpinBox->setRange(0, bgMedianImage.rows);
        roiRadiusSpinBox->setRange(0, std::max(bgMedianImage.cols,bgMedianImage.rows));

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
            case CIRCLE:
                cv::circle(colorMatImage, cv::Point(clampToInt(config.roiCenterX), clampToInt(config.roiCenterY)), clampToInt(config.roiRadius), cv::Scalar(0, 0, 255), 2);
				break;
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
        if (config_.roiType != NONE) {
            cv::bitwise_and(isFg_, inROI_, isFg_);
        }

        // The graded "positive-on-fly" background difference (dBkgd) used by wing tracking
        // is computed on a small box around the fly in segmentWingPixels() (via
        // computeBackgroundDiff), not full-frame here. We only compute the full-frame
        // version below, when DEBUG is on, for the dBkgd.png debug image.
        if (config_.DEBUG && isFirst_) {
            computeBackgroundDiff(cv::Rect(0, 0, currentImage_.cols, currentImage_.rows), dBkgd_);
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
                tmpOutFile = config_.tmpOutDir + QString("\\dBkgd.png");
                printf("Writing difference from background to %s\n", tmpOutFile.toStdString().c_str());
                success = cv::imwrite(tmpOutFile.toStdString(), dBkgd_, imwriteParams_);
                if (!success) printf("Failed writing difference from background to %s\n", tmpOutFile.toStdString().c_str());
                tmpOutFile = config_.tmpOutDir + QString("\\isFg.png");
                printf("Writing foreground mask to %s\n", tmpOutFile.toStdString().c_str());
                success = cv::imwrite(tmpOutFile.toStdString(), isFg_);
                if (!success) printf("Failed writing foreground mask to %s\n", tmpOutFile.toStdString().c_str());
                if (config_.roiType != NONE) {
                    tmpOutFile = config_.tmpOutDir + QString("\\inROI.png");
                    printf("Writing ROI mask to %s\n", tmpOutFile.toStdString().c_str());
                    success = cv::imwrite(tmpOutFile.toStdString(), inROI_);
                    if (!success) printf("Failed writing ROI mask to %s\n", tmpOutFile.toStdString().c_str());
                }
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
    // store the current flyEllipse_ as the (single) previous ellipse
    void FlyTrackPlugin::updateEllipseHistory() {
        // Keep only the last ellipse: updateVelocityHistory uses just .back()/.size(). This used
        // to push_back every frame, growing flyEllipseHistory_ without bound (a memory leak that
        // steadily slowed everything). assign(1,...) reuses the buffer -- no growth, no realloc.
        flyEllipseHistory_.assign(1, flyEllipse_);
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
    // wrap an angle to [-pi, pi) (MATLAB modrange(a,-pi,pi); std::fmod alone is not enough for negatives)
    static double wrapToPi(double a) {
        double r = std::fmod(a + M_PI, 2.0 * M_PI);
        if (r < 0.0) r += 2.0 * M_PI;
        return r - M_PI;
    }

    void FlyTrackPlugin::resolveHeadTail(double wingScoreKeep, double wingScoreFlip, int wingRearPxKeep, int wingRearPxFlip, bool wingValid) {

        double velmag = 0.0;
        double dotprod;
        double costVel0 = 0.0, costVel1 = 0.0;
        double costOri0 = 0.0, costOri1 = 0.0;
        double costWing0 = 0.0, costWing1 = 0.0;
        double cost0 = 0.0, cost1 = 0.0;
        cv::Point2d headDir = cv::Point2d(std::cos(flyEllipse_.theta), std::sin(flyEllipse_.theta));
        cv::Point2d headDirPrev = cv::Point2d(0.0, 0.0);

        // velocity term
        bool velConfident = false;
        if (velocityHistory_.size() > 0) velmag = cv::norm(meanFlyVelocity_);
        if (velmag > config_.minVelocityMagnitude) {
            dotprod = headDir.dot(meanFlyVelocity_) / velmag;
            costVel1 = dotprod;
            costVel0 = -dotprod;
            if (std::abs(dotprod) > MIN_VEL_MATCH_DOTPROD) velConfident = true;
        }

        // wing term: prefer the orientation whose rear half better matches the wing pixels.
        // costWing0 = keep theta, costWing1 = flip theta+pi. Normalized so neutral (~0.5/0.5)
        // when there is little wing evidence -> graceful fallback to velocity/orientation.
        bool wingConfident = false;
        if (wingValid) {
            const double eps = 1e-6;
            double sTot = wingScoreKeep + wingScoreFlip;
            double sKeep = (wingScoreKeep + eps) / (sTot + 2.0 * eps);
            double sFlip = 1.0 - sKeep;
            costWing0 = -sKeep;
            costWing1 = -sFlip;
            // magnitude gate on the unweighted rear-pixel count (so minSingleWingArea stays a
            // pixel count), direction gate on the normalized weighted score (a unitless ratio).
            if ((wingRearPxKeep + wingRearPxFlip) >= config_.minSingleWingArea
                && std::abs(sKeep - sFlip) > MIN_VEL_MATCH_DOTPROD)
                wingConfident = true;
        }

        // diagnostics: unweighted velocity + wing component costs
        flyEllipse_.htVelKeep = costVel0; flyEllipse_.htVelFlip = costVel1;
        flyEllipse_.htWingKeep = costWing0; flyEllipse_.htWingFlip = costWing1;

        // before head/tail has ever been resolved, ignore orientation history; resolve from
        // velocity + wings if either is confident (wings let a stationary fly resolve too).
        if (!headTailResolved_) {
            cost0 = config_.headTailWeightVelocity * costVel0 + config_.headTailWeightWing * costWing0;
            cost1 = config_.headTailWeightVelocity * costVel1 + config_.headTailWeightWing * costWing1;
            flyEllipse_.htOriKeep = costOri0; flyEllipse_.htOriFlip = costOri1; // 0 in bootstrap
            flyEllipse_.htCostKeep = cost0; flyEllipse_.htCostFlip = cost1;
            if (velConfident || wingConfident) {
                if (cost1 < cost0) {
                    flyEllipse_.theta += M_PI;
                    flipFlyOrientationHistory();
                }
                headTailResolved_ = true;
                flyEllipse_.theta = mod2pi(flyEllipse_.theta);
                return;
            }
            // not confident yet -- fall through to also use orientation history (~current orientation)
        }

        // try to match current and previous orientation
        if (orientationHistory_.size() > 0) {
            headDirPrev.x = std::cos(meanFlyOrientation_);
            headDirPrev.y = std::sin(meanFlyOrientation_);
            dotprod = headDir.dot(headDirPrev);
            costOri1 = dotprod;
            costOri0 = -dotprod;
        }

        cost0 = config_.headTailWeightVelocity * costVel0 + config_.headTailWeightWing * costWing0 + costOri0;
        cost1 = config_.headTailWeightVelocity * costVel1 + config_.headTailWeightWing * costWing1 + costOri1;
        flyEllipse_.htOriKeep = costOri0; flyEllipse_.htOriFlip = costOri1;
        flyEllipse_.htCostKeep = cost0; flyEllipse_.htCostFlip = cost1;

        if (cost1 < cost0) {
            // add pi
            flyEllipse_.theta += M_PI;
        }

        // store theta in range -pi, pi
        flyEllipse_.theta = mod2pi(flyEllipse_.theta);
    }

    // Segment wing pixels from the (positive-on-fly) background difference. Orientation-
    // independent, runs once per frame. Mirrors TrackWings_BackSub.m.
    // Compute the positive-on-fly background difference (per flyVsBgMode), ROI-masked, over
    // the given box. CV_8U; saturating subtract clamps negatives to 0 (matches simplewing,
    // which only thresholds positive differences). Used for the small wing box and, full-
    // frame, for the DEBUG dBkgd.png image.
    void FlyTrackPlugin::computeBackgroundDiff(const cv::Rect& box, cv::Mat& dBkgdOut) {
        cv::Mat imBox = currentImage_(box);
        cv::Mat bgBox = bgMedianImage_(box);
        // Raw background difference (absolute). When normalizeWingByBackground is set, divide
        // by the local background brightness so the wing thresholds are invariant to the strong
        // illumination gradient across the arena (bright center vs. dim edge): this is a backlit
        // setup -- the fly attenuates transmitted light multiplicatively -- so (bg-im)/bg is the
        // fraction of light absorbed, ~constant for the same fly regardless of local brightness.
        // Scaled by 255 so dBkgd stays in 0..255 ("fraction absorbed * 255"); divide-by-zero
        // (bg=0 at corners/outside ROI) -> 0. NOTE: wing-tracking only (the body ellipse uses a
        // separate absolute-threshold path); when normalized, the wing thresholds (mindBody /
        // mindWing*) are on this normalized scale, not raw counts.
        cv::Mat diff;
        switch (config_.flyVsBgMode) {
        case FLY_DARKER_THAN_BG:
            cv::subtract(bgBox, imBox, diff);
            break;
        case FLY_BRIGHTER_THAN_BG:
            cv::subtract(imBox, bgBox, diff);
            break;
        case FLY_ANY_DIFFERENCE_BG:
            cv::absdiff(imBox, bgBox, diff);
            break;
        }
        if (config_.normalizeWingByBackground)
            cv::divide(diff, bgBox, dBkgdOut, 255.0, CV_8U); // 255*diff/bg (relative), /0 -> 0
        else
            dBkgdOut = diff;                                 // raw absolute difference
        if (config_.roiType != NONE) {
            cv::bitwise_and(dBkgdOut, inROI_(box), dBkgdOut);
        }
    }

    void FlyTrackPlugin::segmentWingPixels(std::vector<cv::Point>& wingPx) {
        wingPx.clear();
        if (currentImage_.empty() || bgMedianImage_.empty()) return;
        if (flyEllipse_.a <= 0.0) return; // no valid body -> no wings

        int rb = std::max(1, config_.radiusDilateBody);
        int rw = std::max(1, config_.radiusOpenWing);

        // For speed, segment wings only within a bounding box around the pre-tracked fly
        // (body + wings) instead of the whole frame. Wings trail the centroid by ~a body
        // length, so a half-width of WING_BBOX_A_FACTOR * a contains them (shared with the
        // preview zoom). The +margin leaves room for the morphology kernels.
        int margin = rb + rw + 2;
        int half = (int)std::ceil(WING_BBOX_A_FACTOR * flyEllipse_.a) + margin;
        int cx = (int)std::lround(flyEllipse_.x);
        int cy = (int)std::lround(flyEllipse_.y);
        int x0 = std::max(0, cx - half);
        int y0 = std::max(0, cy - half);
        int x1 = std::min(currentImage_.cols, cx + half + 1);
        int y1 = std::min(currentImage_.rows, cy + half + 1);
        if (x1 <= x0 || y1 <= y0) return;
        cv::Rect box(x0, y0, x1 - x0, y1 - y0);
        cv::Mat d; // background difference computed on just this box
        computeBackgroundDiff(box, d);

        cv::Mat seBody = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * rb + 1, 2 * rb + 1));
        cv::Mat seWing = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * rw + 1, 2 * rw + 1));

        // body mask (dilated)
        cv::Mat isBodyThresh = d >= config_.mindBody; // CV_8U 0/255
        cv::Mat isBody;
        cv::dilate(isBodyThresh, isBody, seBody);
        cv::Mat notBody;
        cv::bitwise_not(isBody, notBody);

        // wing hysteresis seeds/mask, excluding (dilated) body
        cv::Mat wingHighThresh = d >= config_.mindWingHigh;
        cv::Mat wingLowThresh = d >= config_.mindWingLow;
        cv::Mat wingHigh, wingLow;
        cv::bitwise_and(wingHighThresh, notBody, wingHigh);
        cv::bitwise_and(wingLowThresh, notBody, wingLow);

        // morphological reconstruction (imreconstruct, 4-conn): keep wingLow components
        // that contain at least one high-threshold seed pixel
        cv::Mat labels;
        int nLabels = cv::connectedComponents(wingLow, labels, 4, CV_32S);
        cv::Mat iswing = cv::Mat::zeros(d.size(), CV_8U);
        if (nLabels > 1) {
            std::vector<unsigned char> keep(nLabels, 0);
            for (int yy = 0; yy < wingHigh.rows; yy++) {
                const unsigned char* hp = wingHigh.ptr<unsigned char>(yy);
                const int* lp = labels.ptr<int>(yy);
                for (int xx = 0; xx < wingHigh.cols; xx++)
                    if (hp[xx] && lp[xx] > 0) keep[lp[xx]] = 1;
            }
            for (int yy = 0; yy < iswing.rows; yy++) {
                const int* lp = labels.ptr<int>(yy);
                unsigned char* wp = iswing.ptr<unsigned char>(yy);
                for (int xx = 0; xx < iswing.cols; xx++)
                    if (lp[xx] > 0 && keep[lp[xx]]) wp[xx] = 255;
            }
        }
        cv::morphologyEx(iswing, iswing, cv::MORPH_OPEN, seWing);
        cv::morphologyEx(iswing, iswing, cv::MORPH_CLOSE, seWing);

        // single fly: all wing-mask pixels belong to this fly.
        // findNonZero gives patch-local coords -> offset back to full-image coords.
        if (cv::countNonZero(iswing) > 0) {
            std::vector<cv::Point> pts;
            cv::findNonZero(iswing, pts);
            wingPx.reserve(pts.size());
            for (size_t i = 0; i < pts.size(); i++)
                wingPx.push_back(cv::Point(pts[i].x + box.x, pts[i].y + box.y));
        }

        // store the segmentation for the live preview overlay (opt-in, so the clone cost is
        // only paid when the overlay is enabled). Box-local labels: 1=body (dilated), 2=wing.
        if (config_.showWingSegmentation) {
            cv::Mat lab = cv::Mat::zeros(d.size(), CV_8U);
            lab.setTo(1, isBody);
            lab.setTo(2, iswing);
            wingSegLabels_ = lab;
            wingSegBox_ = box;
        }

        // DEBUG: dump a visualization of the wing/body segmentation so the thresholds can be
        // inspected (filename has the frame number). Over the raw image box (4x, nearest):
        // body (>=mindBody, dilated) = red; wing pixels actually used = green; yellow = body
        // axis + centroid. By default only the first tracked frame is dumped; pass
        // --debug-seg-all-frames to dump every frame (run short segments -- one PNG/frame).
        if (config_.DEBUG && (debugSegAllFrames_ || isFirst_) && !config_.tmpOutDir.isEmpty()) {
            cv::Mat vis;
            cv::cvtColor(currentImage_(box), vis, cv::COLOR_GRAY2BGR);
            for (int yy = 0; yy < vis.rows; yy++) {
                const unsigned char* bp = isBody.ptr<unsigned char>(yy);
                const unsigned char* wp = iswing.ptr<unsigned char>(yy);
                cv::Vec3b* vp = vis.ptr<cv::Vec3b>(yy);
                for (int xx = 0; xx < vis.cols; xx++) {
                    if (wp[xx]) vp[xx] = cv::Vec3b(0, 200, 0);       // green: wing
                    else if (bp[xx]) vp[xx] = cv::Vec3b(0, 0, 255);  // red: body
                }
            }
            const int SC = 4;
            cv::resize(vis, vis, cv::Size(), SC, SC, cv::INTER_NEAREST);
            double cxv = SC * (flyEllipse_.x - box.x), cyv = SC * (flyEllipse_.y - box.y);
            double axx = std::cos(flyEllipse_.theta), axy = std::sin(flyEllipse_.theta);
            double L = SC * 3.0 * flyEllipse_.a;
            cv::line(vis, cv::Point((int)(cxv - L * axx), (int)(cyv - L * axy)),
                     cv::Point((int)(cxv + L * axx), (int)(cyv + L * axy)),
                     cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
            cv::circle(vis, cv::Point((int)cxv, (int)cyv), 3, cv::Scalar(255, 255, 0), -1);
            QString dbgDir = config_.tmpOutDir + QString("\\wingseg");
            if (!QFile::exists(dbgDir)) QDir().mkdir(dbgDir);
            QString fpath = dbgDir + QString("\\wingseg_%1.png").arg(flyEllipse_.frame, 6, 10, QChar('0'));
            cv::imwrite(fpath.toStdString(), vis, imwriteParams_);
        }
    }

    // Fit wings from the wing-pixel set for one head-orientation hypothesis (headTheta = head).
    // Orientation-dependent, cheap, run once per hypothesis. Mirrors TrackWings_FitWings_Peak.m.
    WingFitResult FlyTrackPlugin::fitWingsFromPixels(const std::vector<cv::Point>& wingPx,
        double x, double y, double headTheta, const FlyTrackConfig& config) {
        WingFitResult r;
        r.angleL = 0.0; r.angleR = 0.0; r.troughAngle = 0.0;
        r.nWings = 0; r.areaL = 0.0; r.areaR = 0.0; r.score = 0.0; r.nRearPx = 0;

        const double maxAngle = config.maxWingPxAngleDeg * M_PI / 180.0;
        const double minNonzero = config.minNonzeroWingAngleDeg * M_PI / 180.0;
        const double rear = headTheta + M_PI;
        const int nBins = std::max(1, config.nBinsDThetaWing);
        const double binWidth = (2.0 * maxAngle) / nBins;

        // bin centers + precomputed per-bin head/tail weight w(c) = cos(c) - cos(maxAngle):
        // peaks (= 1 - cos(maxAngle)) directly behind the head and tapers smoothly to 0 at
        // the +/-maxAngle window edge, non-negative inside. The weights depend only on
        // (nBins, maxAngle), so they are cached across calls (tracking is single-threaded).
        std::vector<double> centers(nBins, 0.0);
        for (int b = 0; b < nBins; b++) centers[b] = -maxAngle + (b + 0.5) * binWidth;
        static std::vector<double> wbin;
        static int wbinNBins = -1;
        static double wbinMaxAngle = -1.0;
        if ((int)wbin.size() != nBins || wbinNBins != nBins || wbinMaxAngle != maxAngle) {
            wbin.assign(nBins, 0.0);
            const double cmax = std::cos(maxAngle);
            for (int b = 0; b < nBins; b++) wbin[b] = std::cos(centers[b]) - cmax;
            wbinNBins = nBins; wbinMaxAngle = maxAngle;
        }

        // 1. per-pixel bearings relative to the rear axis; keep those within the rear window
        //    and bin them (the fit histogram). frac holds raw counts at this stage.
        std::vector<double> dth;
        dth.reserve(wingPx.size());
        std::vector<double> frac(nBins, 0.0);
        for (size_t i = 0; i < wingPx.size(); i++) {
            double d = wrapToPi(std::atan2((double)wingPx[i].y - y, (double)wingPx[i].x - x) - rear);
            if (std::abs(d) <= maxAngle) {
                dth.push_back(d);
                int b = (int)std::floor((d + maxAngle) / binWidth);
                if (b < 0) b = 0;
                if (b >= nBins) b = nBins - 1;
                frac[b] += 1.0;
            }
        }
        const int nwingpx = (int)dth.size();
        r.nRearPx = nwingpx; // unweighted rear-window pixel count (for the head/tail magnitude gate)
        // angle-weighted head/tail score = sum_b count[b]*wbin[b] (one nBins-length dot of the
        // raw histogram, no per-pixel trig). Computed from the counts so keep/flip share the
        // same scale even when one hypothesis is too sparse and early-returns below.
        double scoreSum = 0.0;
        for (int b = 0; b < nBins; b++) scoreSum += frac[b] * wbin[b];
        r.score = scoreSum;
        if (nwingpx <= config.minSingleWingArea) return r; // too few wing pixels (MATLAB: locs sought only if > )

        // 2. normalize the histogram (counts binned above) + smoothing
        for (int b = 0; b < nBins; b++) frac[b] /= (double)nwingpx;
        const std::vector<double>& filt = config.wingFracFilter;
        const int fLen = (int)filt.size();
        const int fHalf = fLen / 2;
        std::vector<double> sf(nBins, 0.0);
        for (int b = 0; b < nBins; b++) {
            double s = 0.0;
            for (int k = 0; k < fLen; k++) {
                int idx = b + k - fHalf;
                if (idx >= 0 && idx < nBins) s += frac[idx] * filt[k];
            }
            sf[b] = s;
        }

        // 3. find up to two peaks (0-based bins)
        int loc1 = 0; double pk = sf[0];
        for (int b = 1; b < nBins; b++) if (sf[b] > pk) { pk = sf[b]; loc1 = b; }
        if (pk < config.wingMinPeakThresholdFrac) return r; // no primary peak -> 0 wings
        int loc2 = -1;
        {
            int bestb = -1; double bestv = -1.0;
            for (int b = 0; b < nBins; b++) {
                bool gtLeft = (b == 0) || (sf[b] > sf[b - 1]);
                bool geRight = (b == nBins - 1) || (sf[b] >= sf[b + 1]);
                if (!(gtLeft && geRight)) continue;                       // local maximum
                if (b >= loc1 - config.wingMinPeakDistBins && b <= loc1 + config.wingMinPeakDistBins) continue;
                if (sf[b] > bestv) { bestv = sf[b]; bestb = b; }
            }
            double peak2MinFrac = config.wingPeakMinFracFactor / (double)nBins;
            if (bestb >= 0 && bestv >= peak2MinFrac) loc2 = bestb;
        }

        std::vector<double> wingAngles, area;
        int npeaks;
        double troughAngle = 0.0;

        if (loc2 < 0) {
            // single peak: angle = median of bearings
            std::vector<double> tmp = dth;
            std::sort(tmp.begin(), tmp.end());
            double med = (nwingpx % 2 == 1) ? tmp[nwingpx / 2]
                                            : 0.5 * (tmp[nwingpx / 2 - 1] + tmp[nwingpx / 2]);
            wingAngles.push_back(med);
            area.push_back((double)nwingpx);
            npeaks = 1;
            troughAngle = med;
        } else {
            // two peaks
            npeaks = 2;
            wingAngles.push_back(centers[loc1]);
            wingAngles.push_back(centers[loc2]);
            int locs[2] = { loc1, loc2 };
            int radius = config.wingRadiusQuadfitBins;
            for (int j = 0; j < 2; j++) {
                std::vector<int> xs;
                for (int d = -radius; d <= radius; d++) {
                    int xb = locs[j] + d;
                    if (xb >= 0 && xb < nBins) xs.push_back(xb);
                }
                int ncurr = (int)xs.size();
                if (ncurr < 3) continue;
                cv::Mat X(ncurr, 3, CV_64F), yv(ncurr, 1, CV_64F);
                for (int i = 0; i < ncurr; i++) {
                    double xb = (double)xs[i];
                    X.at<double>(i, 0) = 1.0; X.at<double>(i, 1) = xb; X.at<double>(i, 2) = xb * xb;
                    yv.at<double>(i, 0) = sf[xs[i]];
                }
                cv::Mat coeffs;
                if (!cv::solve(X, yv, coeffs, cv::DECOMP_SVD)) continue;
                double c2 = coeffs.at<double>(1, 0), c3 = coeffs.at<double>(2, 0);
                if (c3 >= 0) continue; // must be concave
                double maxx = -c2 / (2.0 * c3);
                if (std::abs(maxx - locs[j]) > 1.0 || maxx > nBins - 1 || maxx < 0) continue;
                int fl = (int)std::floor(maxx);
                int cl = (int)std::ceil(maxx);
                if (fl < 0) fl = 0; if (cl > nBins - 1) cl = nBins - 1;
                double wceil = maxx - std::floor(maxx);
                wingAngles[j] = centers[fl] * (1.0 - wceil) + wceil * centers[cl];
            }

            // can't have two wings on the same side of the body
            if (((wingAngles[0] >= 0.0) == (wingAngles[1] >= 0.0)) &&
                std::min(std::abs(wingAngles[0]), std::abs(wingAngles[1])) >= minNonzero) {
                wingAngles.pop_back();
                area.push_back((double)nwingpx);
                npeaks = 1;
            } else {
                // trough between the two peaks (1-based port of TrackWings_FitWings_Peak.m:89-115)
                int minloc = std::min(loc1, loc2) + 1;
                int maxloc = std::max(loc1, loc2) + 1;
                auto SF = [&](int i1) { int idx = i1 - 1; if (idx < 0) idx = 0; if (idx >= nBins) idx = nBins - 1; return sf[idx]; };
                auto CTR = [&](int i1) { int idx = i1 - 1; if (idx < 0) idx = 0; if (idx >= nBins) idx = nBins - 1; return centers[idx]; };
                int troughloc = minloc; double tmin = SF(minloc);
                for (int i = minloc; i <= maxloc; i++) if (SF(i) < tmin) { tmin = SF(i); troughloc = i; }
                int j_last = -1;
                for (int i = minloc; i <= troughloc; i++) if (SF(i) > SF(troughloc)) j_last = (i - minloc + 1);
                double loc1b = (j_last < 0) ? (double)troughloc : (double)(j_last + minloc);
                int j_first = -1;
                for (int i = troughloc; i <= maxloc; i++) if (SF(i) > SF(troughloc)) { j_first = (i - troughloc + 1); break; }
                double loc2b = (j_first < 0) ? (double)troughloc : (double)(j_first + troughloc - 2);
                double troughF = (loc1b + loc2b) / 2.0;
                double areaFrac;
                if (std::fmod(troughF, 1.0) > 0.0) {
                    int k = (int)(troughF - 0.5);
                    double s = 0.0; for (int i = 1; i <= k; i++) s += SF(i);
                    areaFrac = s;
                    troughAngle = (CTR((int)(troughF - 0.5)) + CTR((int)(troughF + 0.5))) / 2.0;
                } else {
                    int T = (int)troughF;
                    double s = 0.0; for (int i = 1; i <= T - 1; i++) s += SF(i);
                    s += SF(T) / 2.0;
                    areaFrac = s;
                    troughAngle = CTR(T);
                }
                area.push_back(areaFrac * nwingpx);
                area.push_back((1.0 - areaFrac) * nwingpx);

                if (area[0] < config.minSingleWingArea && area[1] < config.minSingleWingArea) {
                    wingAngles.clear(); wingAngles.push_back(0.0);
                    area.clear(); area.push_back(0.0);
                    npeaks = 0; troughAngle = 0.0;
                } else if (area[0] < config.minSingleWingArea) {
                    int removei = (wingAngles[0] <= wingAngles[1]) ? 0 : 1; // remove min-angle wing
                    wingAngles.erase(wingAngles.begin() + removei);
                    area.erase(area.begin());                                // drop area(1)
                    npeaks = 1;
                } else if (area[1] < config.minSingleWingArea) {
                    int removei = (wingAngles[0] >= wingAngles[1]) ? 0 : 1; // remove max-angle wing
                    wingAngles.erase(wingAngles.begin() + removei);
                    area.erase(area.begin() + 1);                            // drop area(2)
                    npeaks = 1;
                }
            }
        }

        // sort ascending; pad single-wing result to two entries
        std::sort(wingAngles.begin(), wingAngles.end());
        if ((int)wingAngles.size() == 1) {
            double a0 = wingAngles[0];
            double ar0 = area.empty() ? 0.0 : area[0];
            if (std::abs(a0) > minNonzero) {
                troughAngle = 0.0;
                if (0.0 <= a0) { wingAngles = { 0.0, a0 }; area = { 0.0, ar0 }; }
                else { wingAngles = { a0, 0.0 }; area = { ar0, 0.0 }; }
            } else {
                wingAngles = { a0, a0 };
                area = { ar0, ar0 };
            }
        }

        r.angleL = wingAngles[0];
        r.angleR = wingAngles[1];
        r.nWings = npeaks;
        r.areaL = area[0];
        r.areaR = area[1];
        r.troughAngle = troughAngle;
        return r;
    }

    // Segment wings once, fit both head-orientation hypotheses, resolve head/tail using the
    // wing scores (+ velocity/orientation), and store the chosen fit into flyEllipse_.
    void FlyTrackPlugin::trackWings() {
        double theta0 = flyEllipse_.theta;
        segmentWingPixels(wingPx_);
        WingFitResult wfKeep = fitWingsFromPixels(wingPx_, flyEllipse_.x, flyEllipse_.y, theta0, config_);
        WingFitResult wfFlip = fitWingsFromPixels(wingPx_, flyEllipse_.x, flyEllipse_.y, theta0 + M_PI, config_);

        // diagnostic: keep vs flip wing-fit scores used for head/tail resolution
        flyEllipse_.htScoreKeep = wfKeep.score;
        flyEllipse_.htScoreFlip = wfFlip.score;

        resolveHeadTail(wfKeep.score, wfFlip.score, wfKeep.nRearPx, wfFlip.nRearPx, true);

        // pick the fit matching the resolved orientation (flipped if theta moved ~pi from theta0)
        bool flipped = std::abs(mod2pi(flyEllipse_.theta - theta0)) > (M_PI / 2.0);
        const WingFitResult& wf = flipped ? wfFlip : wfKeep;
        flyEllipse_.wingAngleL = wf.angleL;
        flyEllipse_.wingAngleR = wf.angleR;
        flyEllipse_.wingTroughAngle = wf.troughAngle;
        flyEllipse_.nWingsDetected = wf.nWings;
        flyEllipse_.wingAreaL = wf.areaL;
        flyEllipse_.wingAreaR = wf.areaR;
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

    // helper function

    // savely cast a double to a int
    // in theory, cv::saturated_cast<int>(value) should have done the job, but it didn't work for a very large double
    int FlyTrackPlugin::clampToInt(double value) {
        if (value > std::numeric_limits<int>::max()) {
            return std::numeric_limits<int>::max();
        }
        else if (value < std::numeric_limits<int>::min()) {
            return std::numeric_limits<int>::min();
        }
        return static_cast<int>(value);
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
    void fitEllipse(cv::Mat& isFg, EllipseParams& flyEllipse) {

        // eigen decomposition of covariance matrix
        // this probably isn't the fastest way to do this, but
        // it seems to work
        cv::Mat fgPixels;
        int maxSize = static_cast<int>(isFg.total());
        cv::findNonZero(isFg, fgPixels);
        if (fgPixels.rows == 0 || fgPixels.rows == maxSize) {
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
        if (std::isnan(flyEllipse.a)) { flyEllipse.a = 0.0; }
        flyEllipse.b = std::sqrt(lambda2) * 2.0;
        if (std::isnan(flyEllipse.b)) { flyEllipse.b = 0.0; } // Workaround in case lambda2 is "-0.0" or large negative number

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
        json += QString("\"timestamp\": %1,").arg(QDateTime::currentMSecsSinceEpoch());
        json += QString("\"capture_time\": %1,").arg(ell.captureTime, 0, 'f', 6);
        json += QString("\"x\": %1,").arg(ell.x);
        json += QString("\"y\": %1,").arg(ell.y);
        json += QString("\"a\": %1,").arg(ell.a);
        json += QString("\"b\": %1,").arg(ell.b);
        json += QString("\"theta\": %1,").arg(ell.theta);
        json += QString("\"wing_anglel\": %1,").arg(ell.wingAngleL);
        json += QString("\"wing_angler\": %1,").arg(ell.wingAngleR);
        json += QString("\"wing_trough_angle\": %1,").arg(ell.wingTroughAngle);
        json += QString("\"nwings\": %1,").arg(ell.nWingsDetected);
        json += QString("\"wing_areal\": %1,").arg(ell.wingAreaL);
        json += QString("\"wing_arear\": %1").arg(ell.wingAreaR);
        json += QString("}");
        return json;
    }

}
