#ifndef FLYTRACK_CONFIG_HPP
#define FLYTRACK_CONFIG_HPP

#include "rtn_status.hpp"
#include <QVariantMap>
#include <QColor>


namespace bias
{

    const int N_ROI_TYPES = 1;
    enum ROIType {RECTANGLE};

    const int N_FLY_VS_BG_MODES = 3;
    enum FlyVsBgModeType { FLY_DARKER_THAN_BG, FLY_BRIGHTER_THAN_BG, FLY_ANY_DIFFERENCE_BG };

    bool roiTypeToString(ROIType roiType, QString& roiTypeString);
    bool roiTypeFromString(QString roiTypeString, ROIType& roiType);
    bool flyVsBgModeToString(FlyVsBgModeType flyVsBgMode, QString& flyVsBgModeString);
    bool flyVsBgModeFromString(QString flyVsBgModeString, FlyVsBgModeType& flyVsBgMode);

    class FlyTrackConfig
    {

        public:
            // default parameters
            static const QString DEFAULT_BG_IMAGE_FILE_PATH; // saved background median estimate
            static const QString DEFAULT_TMP_OUT_DIR; // temporary output directory

            static const int DEFAULT_BACKGROUND_THRESHOLD; // foreground/background threshold, between 0 and 255
            static const int DEFAULT_N_FRAMES_SKIP_BG_EST; // number of frames skipped between frames added to background model
            static const FlyVsBgModeType DEFAULT_FLY_VS_BG_MODE; // whether the fly is darker than the background
            static const ROIType DEFAULT_ROI_TYPE; // type of ROI
            static const double DEFAULT_ROI_CENTER_X_FRAC; // x-coordinate of ROI center, relative
            static const double DEFAULT_ROI_CENTER_Y_FRAC; // y-coordinate of ROI center, relative
            static const double DEFAULT_ROI_RADIUS_FRAC; // radius of ROI, relative
            static const int DEFAULT_HISTORY_BUFFER_LENGTH; // number of frames to buffer velocity, orientation
            static const int DEFAULT_MAX_TRACK_QUEUE_LENGTH; // maximum number of track frames to buffer
            static const double DEFAULT_MIN_VELOCITY_MAGNITUDE; // minimum velocity magnitude in pixels/frame to consider fly moving
            static const double DEFAULT_HEAD_TAIL_WEIGHT_VELOCITY; // weight of velocity dot product in head-tail orientation resolution
            static const double DEFAULT_MIN_VEL_MATCH_DOTPROD; // minimum dot product for velocity matching
            static const bool DEFAULT_DEBUG; // flag for debugging
            static const bool DEFAULT_COMPUTE_BG_MODE; // flag of whether to compute the background (true) when camera is running or track a fly (false)


            // parameters
            bool computeBgMode; // flag of whether to compute the background (true) when camera is running or track a fly (false)
            QString bgImageFilePath; // saved background median estimate
            QString tmpOutDir; // temporary output directory
            QString logFilePath; // log file path
            int backgroundThreshold; // foreground threshold
            int nFramesSkipBgEst; // number of frames to skip between frames added to background model
            FlyVsBgModeType flyVsBgMode; // whether the fly is darker than the background
            ROIType roiType; // type of ROI
            double triangleCenterX; // x-coordinate of ROI center
            double triangleCenterY; // y-coordinate of ROI center
            double triangleSide; // radius of ROI
            double triangleAngle;
			double innerOffsetAlong; // x-coordinate of left feeder ROI center
			double innerOffsetOut; // y-coordinate of left feeder ROI center
			double innerWidth; // radius of left feeder ROI
			double innerHeight; // radius of left feeder ROI
			double roiRightFeederCenterX; // x-coordinate of right feeder ROI center
			double roiRightFeederCenterY; // y-coordinate of right feeder ROI center
			double roiRightFeederWidth; // radius of right feeder ROI
			double roiRightFeederHeight; // radius of right feeder ROI
			double roiResetCenterX; // x-coordinate of reset ROI center
			double roiResetCenterY; // y-coordinate of reset ROI center
			double roiResetWidth; // radius of reset ROI
			double outerDepth; // radius of reset ROI
            int historyBufferLength; // number of frames to buffer velocity, orientation
            int maxTrackQueueLength; // number of tracks to buffer
            double minVelocityMagnitude; // minimum velocity magnitude in pixels/frame to consider fly moving
            double headTailWeightVelocity; // weight of velocity dot product in head-tail orientation resolution
            bool DEBUG; // flag for debugging
            QString trackFileName; // relative name of output track file
            QString tmpTrackFilePath; // absolute path of track file -- not stored in config file

            FlyTrackConfig();
            FlyTrackConfig FlyTrackConfig::copy();
			void setRoiParams(ROIType roiTypeNew, double triangleCenterXNew, double triangleCenterYNew, double triangleSideNew, double triangleAngleNew, double innerOffsetAlongNew, double innerOffsetOutNew, double innerWidthNew, double innerHeightNew, double roiRightFeederCenterXNew, double roiRightFeederCenterYNew, double roiRightFeederWidthNew, double roiRightFeederHeightNew, double roiResetCenterX, double roiResetCenterY, double roiResetWidth, double outerDepth);

            RtnStatus setBgEstFromMap(QVariantMap configMap);
            RtnStatus setRoiFromMap(QVariantMap configMap);
            RtnStatus setBgSubFromMap(QVariantMap configMap);
            RtnStatus setHeadTailFromMap(QVariantMap configMap);
            RtnStatus setMiscFromMap(QVariantMap configMap);
            QVariantMap toMap();
            RtnStatus fromMap(QVariantMap configMap);
            RtnStatus fromJson(QByteArray jsonConfigArray);
            QByteArray toJson();
            QString toString();
            bool trackFilePathSet();
            bool trackFileNameSet();

            void print();

    };
}

#endif