#include "flytrack_config.hpp"
#include "json.hpp"
#include <iostream>
#include <QMessageBox>
#include <QtDebug>
#include <QFileInfo>

namespace bias
{

    const QString FlyTrackConfig::DEFAULT_BG_IMAGE_FILE_PATH = QString(""); // saved background median estimate
    const QString FlyTrackConfig::DEFAULT_TMP_OUT_DIR = QString("tmp"); // temporary output directory
    const int FlyTrackConfig::DEFAULT_BACKGROUND_THRESHOLD = 75; // foreground/background threshold, between 0 and 255
    const int FlyTrackConfig::DEFAULT_N_FRAMES_SKIP_BG_EST = 500; // number of frames used for background estimation, set to 0 to use all frames
    const FlyVsBgModeType FlyTrackConfig::DEFAULT_FLY_VS_BG_MODE = FLY_DARKER_THAN_BG; // whether the fly is darker than the background
    const ROIType FlyTrackConfig::DEFAULT_ROI_TYPE = RECTANGLE; // type of ROI
    const int FlyTrackConfig::DEFAULT_HISTORY_BUFFER_LENGTH = 5; // number of frames to buffer velocity, orientation
    const int FlyTrackConfig::DEFAULT_MAX_TRACK_QUEUE_LENGTH = 10000; // maximum number of track frames to buffer
    const double FlyTrackConfig::DEFAULT_MIN_VELOCITY_MAGNITUDE = 1.0; // minimum velocity magnitude in pixels/frame to consider fly moving
    const double FlyTrackConfig::DEFAULT_HEAD_TAIL_WEIGHT_VELOCITY = 3.0; // weight of velocity dot product in head-tail orientation resolution
    const double FlyTrackConfig::DEFAULT_MIN_VEL_MATCH_DOTPROD = 0.25; // minimum dot product for velocity matching
    const bool FlyTrackConfig::DEFAULT_DEBUG = false; // flag for debugging
    const bool FlyTrackConfig::DEFAULT_COMPUTE_BG_MODE = false; // flag of whether to compute the background (true) when camera is running or track a fly (false)

    FlyTrackConfig::FlyTrackConfig()
    {
        computeBgMode = DEFAULT_COMPUTE_BG_MODE;
        bgImageFilePath = DEFAULT_BG_IMAGE_FILE_PATH;
        tmpOutDir = DEFAULT_TMP_OUT_DIR;
        backgroundThreshold = DEFAULT_BACKGROUND_THRESHOLD;
        nFramesSkipBgEst = DEFAULT_N_FRAMES_SKIP_BG_EST;
        flyVsBgMode = DEFAULT_FLY_VS_BG_MODE;
        roiType = DEFAULT_ROI_TYPE;
        historyBufferLength = DEFAULT_HISTORY_BUFFER_LENGTH;
        maxTrackQueueLength = DEFAULT_MAX_TRACK_QUEUE_LENGTH;
        minVelocityMagnitude = DEFAULT_MIN_VELOCITY_MAGNITUDE;
        headTailWeightVelocity = DEFAULT_HEAD_TAIL_WEIGHT_VELOCITY;
        DEBUG = DEFAULT_DEBUG;
        triangleCenterX = 100; // Add this to defaults
        triangleCenterY = 100; // Add this to defaults
        triangleSide = 100; // Add this to defaults
        triangleAngle = 100; // Add this to defaults
        innerOffsetAlong = 100; // Add this to defaults
        innerOffsetOut = 100; // Add this to defaults
        innerWidth = 100; // Add this to defaults
        innerHeight = 100; // Add this to defaults
        roiRightFeederCenterX = 100; // Add this to defaults
        roiRightFeederCenterY = 100; // Add this to defaults
        roiRightFeederWidth = 100; // Add this to defaults
        roiRightFeederHeight = 100; // Add this to defaults
		roiResetCenterX = 100; // Add this to defaults
		roiResetCenterY = 100; // Add this to defaults
		roiResetWidth = 100; // Add this to defaults
		outerDepth = 100; // Add this to defaults


        trackFileName = QString(""); // empty string means it is not set
        tmpTrackFilePath = QString(""); // empty string means it is not set
    }


    FlyTrackConfig FlyTrackConfig::copy() {
        FlyTrackConfig config;
        config.computeBgMode = computeBgMode;
        config.bgImageFilePath = bgImageFilePath;
        config.tmpOutDir = tmpOutDir;
        config.backgroundThreshold = backgroundThreshold;
        config.nFramesSkipBgEst = nFramesSkipBgEst;
        config.flyVsBgMode = flyVsBgMode;
        config.roiType = roiType;
        config.historyBufferLength = historyBufferLength;
        config.maxTrackQueueLength = maxTrackQueueLength;
        config.minVelocityMagnitude = minVelocityMagnitude;
        config.headTailWeightVelocity = headTailWeightVelocity;
        config.DEBUG = DEBUG;
        config.triangleCenterX = triangleCenterX;
        config.triangleCenterY = triangleCenterY;
        config.triangleSide = triangleSide;
        config.triangleAngle = triangleAngle;
		config.innerOffsetAlong = innerOffsetAlong;
		config.innerOffsetOut = innerOffsetOut;
		config.innerWidth = innerWidth;
        config.innerHeight = innerHeight;
		config.roiRightFeederCenterX = roiRightFeederCenterX;
		config.roiRightFeederCenterY = roiRightFeederCenterY;
		config.roiRightFeederWidth = roiRightFeederWidth;
		config.roiRightFeederHeight = roiRightFeederHeight;
		config.roiResetCenterX = roiResetCenterX;
		config.roiResetCenterY = roiResetCenterY;
		config.roiResetWidth = roiResetWidth;
		config.outerDepth = outerDepth;

        config.trackFileName = trackFileName;
        config.tmpTrackFilePath = tmpTrackFilePath;
        return config;

    }

    QString FlyTrackConfig::toString() {
        QString configStr;
        configStr += QString("computeBgMode: %1\n").arg(computeBgMode);
        configStr += QString("bgImageFilePath: %1\n").arg(bgImageFilePath);
        configStr += QString("tmpOutDir: %1\n").arg(tmpOutDir);
        configStr += QString("trackFileName: %1\n").arg(trackFileName);
        configStr += QString("tmpTrackFilePath: %1\n").arg(tmpTrackFilePath);
        configStr += QString("backgroundThreshold: %1\n").arg(backgroundThreshold);
        configStr += QString("nFramesSkipBgEst: %1\n").arg(nFramesSkipBgEst);
        configStr += QString("flyVsBgMode: %1\n").arg(flyVsBgMode);
        QString roiTypeString;
        roiTypeToString(roiType, roiTypeString);
        configStr += QString("roiType: %1\n").arg(roiTypeString);
        configStr += QString("triangleCenterX: %1\n").arg(triangleCenterX);
        configStr += QString("triangleCenterY: %1\n").arg(triangleCenterY);
        configStr += QString("triangleSide: %1\n").arg(triangleSide);
        configStr += QString("triangleAngle: %1\n").arg(triangleAngle);
        configStr += QString("innerOffsetAlong: %1\n").arg(innerOffsetAlong);
        configStr += QString("innerOffsetOut: %1\n").arg(innerOffsetOut);
        configStr += QString("roiRightFeederCenterX: %1\n").arg(roiRightFeederCenterX);
        configStr += QString("roiRightFeederCenterY: %1\n").arg(roiRightFeederCenterY);
        configStr += QString("innerWidth: %1\n").arg(innerWidth);
        configStr += QString("innerHeight: %1\n").arg(innerHeight);
        configStr += QString("roiRightFeederWidth: %1\n").arg(roiRightFeederWidth);
        configStr += QString("roiRightFeederHeight: %1\n").arg(roiRightFeederHeight);
		configStr += QString("roiResetCenterX: %1\n").arg(roiResetCenterX);
		configStr += QString("roiResetCenterY: %1\n").arg(roiResetCenterY);
		configStr += QString("roiResetWidth: %1\n").arg(roiResetWidth);
		configStr += QString("outerDepth: %1\n").arg(outerDepth);
        configStr += QString("historyBufferLength: %1\n").arg(historyBufferLength);
        configStr += QString("maxTrackQueueLength: %1\n").arg(maxTrackQueueLength);
        configStr += QString("minVelocityMagnitude: %1\n").arg(minVelocityMagnitude);
        configStr += QString("headTailWeightVelocity: %1\n").arg(headTailWeightVelocity);
        configStr += QString("DEBUG: %1\n").arg(DEBUG);
        return configStr;

    }

    void FlyTrackConfig::print() {
        std::cout << toString().toStdString();
    }

    bool FlyTrackConfig::trackFilePathSet() {
        return !tmpTrackFilePath.isEmpty();
    }
    bool FlyTrackConfig::trackFileNameSet() {
        return !trackFileName.isEmpty();
    }


    RtnStatus FlyTrackConfig::fromMap(QVariantMap configMap) {
        RtnStatus rtnStatus;
        rtnStatus.success = true;

        QVariantMap oldConfigMap = toMap();
        RtnStatus rtnStatusBgEst = setBgEstFromMap(configMap["bgEst"].toMap());
        RtnStatus rtnStatusRoi = setRoiFromMap(configMap["roi"].toMap());
        RtnStatus rtnStatusBgSub = setBgSubFromMap(configMap["bgSub"].toMap());
        RtnStatus rtnStatusHeadTail = setHeadTailFromMap(configMap["headTail"].toMap());
        RtnStatus rtnStatusMisc = setMiscFromMap(configMap["misc"].toMap());

        rtnStatus.success = rtnStatusBgEst.success && rtnStatusRoi.success && rtnStatusBgSub.success && rtnStatusHeadTail.success;
        rtnStatus.message += rtnStatusBgEst.message + QString(", ");
        rtnStatus.message += rtnStatusRoi.message + QString(", ");
        rtnStatus.message += rtnStatusBgSub.message + QString(", ");
        rtnStatus.message += rtnStatusHeadTail.message + QString(", ");
        rtnStatus.message += rtnStatusMisc.message;

        return rtnStatus;

    }

    RtnStatus FlyTrackConfig::setBgEstFromMap(QVariantMap configMap) {
        RtnStatus rtnStatus;
        rtnStatus.success = true;
        rtnStatus.message = QString("");

        if (configMap.isEmpty())
        {
            rtnStatus.message = QString("flyTrack bgEst config empty");
            return rtnStatus;
        }
        if (configMap.contains("computeBgMode")) {
            if (configMap["computeBgMode"].canConvert<bool>())
                computeBgMode = configMap["computeBgMode"].toBool();
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage("unable to convert computeBgMode to bool");
            }
        }
        if (configMap.contains("bgImageFilePath")) {
            if (configMap["bgImageFilePath"].canConvert<QString>())
                bgImageFilePath = configMap["bgImageFilePath"].toString();
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage("unable to convert bgImageFilePath to string");
            }
        }
        if (configMap.contains("nFramesSkipBgEst")) {
            if (configMap["nFramesSkipBgEst"].canConvert<int>())
                nFramesSkipBgEst = configMap["nFramesSkipBgEst"].toInt();
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage("unable to convert nFramesSkipBgEst to int");
            }
        }
        return rtnStatus;
    }

    RtnStatus FlyTrackConfig::setRoiFromMap(QVariantMap configMap) {
        RtnStatus rtnStatus;
        rtnStatus.success = true;
        rtnStatus.message = QString("");

        if (configMap.isEmpty())
        {
            rtnStatus.message = QString("flyTrack roi config empty");
            return rtnStatus;
        }

        if (configMap.contains("roiType")) {
            if (configMap["roiType"].canConvert<QString>()) {
                QString roiTypeStr = configMap["roiType"].toString();
                bool success = roiTypeFromString(roiTypeStr, roiType);
                if (!success) {
                    rtnStatus.success = false;
                    rtnStatus.appendMessage(QString("unknown roiType %1").arg(roiTypeStr));
                }
            }
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage("unable to convert roiType to string");
            }
        }

        if (configMap.contains("triangleCenterX")) {
            if (configMap["triangleCenterX"].canConvert<double>()) {
                triangleCenterX = configMap["triangleCenterX"].toDouble();
            }
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage(QString("unable to convert triangleCenterX to double"));
            }
        }

        if (configMap.contains("triangleCenterY")) {
            if (configMap["triangleCenterY"].canConvert<double>()) {
                triangleCenterY = configMap["triangleCenterY"].toDouble();
            }
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage(QString("unable to convert triangleCenterY to double"));
            }
        }

        if (configMap.contains("roiRadius")) {
            if (configMap["roiRadius"].canConvert<double>()) {
                triangleSide = configMap["roiRadius"].toDouble();
            }
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage(QString("unable to convert roiRadius to double"));
            }
        }

        if (configMap.contains("triangleSide")) {
            if (configMap["triangleSide"].canConvert<double>()) {
                triangleSide = configMap["triangleSide"].toDouble();
            }
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage(QString("unable to convert triangleSide to double"));
            }
        }

        if (configMap.contains("triangleAngle")) {
            if (configMap["triangleAngle"].canConvert<double>()) {
                triangleAngle = configMap["triangleAngle"].toDouble();
            }
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage(QString("unable to convert triangleAngle to double"));
            }
        }

        if (configMap.contains("innerOffsetAlong")) {
            if (configMap["innerOffsetAlong"].canConvert<double>()) {
                innerOffsetAlong = configMap["innerOffsetAlong"].toDouble();
            }
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage(QString("unable to convert innerOffsetAlong to double"));
            }
        }

        if (configMap.contains("innerOffsetOut")) {
            if (configMap["innerOffsetOut"].canConvert<double>()) {
                innerOffsetOut = configMap["innerOffsetOut"].toDouble();
            }
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage(QString("unable to convert innerOffsetOut to double"));
            }
        }

        if (configMap.contains("innerWidth")) {
            if (configMap["innerWidth"].canConvert<double>()) {
                innerWidth = configMap["innerWidth"].toDouble();
            }
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage(QString("unable to convert innerWidth to double"));
            }
        }

        if (configMap.contains("innerHeight")) {
            if (configMap["innerHeight"].canConvert<double>()) {
                innerHeight = configMap["innerHeight"].toDouble();
            }
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage(QString("unable to convert innerHeight to double"));
            }
        }

        if (configMap.contains("roiRightFeederCenterX")) {
            if (configMap["roiRightFeederCenterX"].canConvert<double>()) {
                roiRightFeederCenterX = configMap["roiRightFeederCenterX"].toDouble();
            }
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage(QString("unable to convert roiRightFeederCenterX to double"));
            }
        }

        if (configMap.contains("roiRightFeederCenterY")) {
            if (configMap["roiRightFeederCenterY"].canConvert<double>()) {
                roiRightFeederCenterY = configMap["roiRightFeederCenterY"].toDouble();
            }
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage(QString("unable to convert roiRightFeederCenterY to double"));
            }
        }

        if (configMap.contains("roiRightFeederWidth")) {
            if (configMap["roiRightFeederWidth"].canConvert<double>()) {
                roiRightFeederWidth = configMap["roiRightFeederWidth"].toDouble();
            }
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage(QString("unable to convert roiRightFeederWidth to double"));
            }
        }

        if (configMap.contains("roiRightFeederHeight")) {
            if (configMap["roiRightFeederHeight"].canConvert<double>()) {
                roiRightFeederHeight = configMap["roiRightFeederHeight"].toDouble();
            }
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage(QString("unable to convert roiRightFeederHeight to double"));
            }
        }
		if (configMap.contains("roiResetCenterX")) {
			if (configMap["roiResetCenterX"].canConvert<double>()) {
				roiResetCenterX = configMap["roiResetCenterX"].toDouble();
			}
			else {
				rtnStatus.success = false;
				rtnStatus.appendMessage(QString("unable to convert roiResetCenterX to double"));
			}
		}
		if (configMap.contains("roiResetCenterY")) {
			if (configMap["roiResetCenterY"].canConvert<double>()) {
				roiResetCenterY = configMap["roiResetCenterY"].toDouble();
			}
			else {
				rtnStatus.success = false;
				rtnStatus.appendMessage(QString("unable to convert roiResetCenterY to double"));
			}
		}
		if (configMap.contains("roiResetWidth")) {
			if (configMap["roiResetWidth"].canConvert<double>()) {
				roiResetWidth = configMap["roiResetWidth"].toDouble();
			}
			else {
				rtnStatus.success = false;
				rtnStatus.appendMessage(QString("unable to convert roiResetWidth to double"));
			}
		}
		if (configMap.contains("outerDepth")) {
			if (configMap["outerDepth"].canConvert<double>()) {
				outerDepth = configMap["outerDepth"].toDouble();
			}
			else {
				rtnStatus.success = false;
				rtnStatus.appendMessage(QString("unable to convert outerDepth to double"));
			}
		}


        return rtnStatus;
    }

    RtnStatus FlyTrackConfig::setBgSubFromMap(QVariantMap configMap) {

        RtnStatus rtnStatus;
        rtnStatus.success = true;
        rtnStatus.message = QString("");

        if (configMap.isEmpty())
        {
            rtnStatus.message = QString("flyTrack bgSub config empty");
            return rtnStatus;
        }

        if (configMap.contains("backgroundThreshold")) {
            if (configMap["backgroundThreshold"].canConvert<int>())
                backgroundThreshold = configMap["backgroundThreshold"].toInt();
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage("unable to convert backgroundThreshold to int");
            }
        }
        if (configMap.contains("flyVsBgMode")) {
            if (configMap["flyVsBgMode"].canConvert<QString>()) {
                QString flyVsBgModeStr = configMap["flyVsBgMode"].toString();
                if (flyVsBgModeStr == "FLY_DARKER_THAN_BG") flyVsBgMode = FLY_DARKER_THAN_BG;
                else if (flyVsBgModeStr == "FLY_BRIGHTER_THAN_BG") flyVsBgMode = FLY_BRIGHTER_THAN_BG;
                else if (flyVsBgModeStr == "FLY_ANY_DIFFERENCE_BG") flyVsBgMode = FLY_ANY_DIFFERENCE_BG;
                else {
                    rtnStatus.success = false;
                    rtnStatus.appendMessage("unable to parse flyVsBgMode");
                }
            }
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage("unable to convert flyVsBgMode to string");
            }
        }
        return rtnStatus;

    }

    RtnStatus FlyTrackConfig::setHeadTailFromMap(QVariantMap configMap) {
        RtnStatus rtnStatus;
        rtnStatus.success = true;
        rtnStatus.message = QString("");
        if (configMap.isEmpty())
        {
            rtnStatus.message = QString("flyTrack headTail config empty");
            return rtnStatus;
        }
        if (configMap.contains("historyBufferLength")) {
            if (configMap["historyBufferLength"].canConvert<int>())
                historyBufferLength = configMap["historyBufferLength"].toInt();
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage("unable to convert historyBufferLength to int");
            }
        }
        if (configMap.contains("maxTrackQueueLength")) {
            if (configMap["maxTrackQueueLength"].canConvert<int>())
                maxTrackQueueLength = configMap["maxTrackQueueLength"].toInt();
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage("unable to convert maxTrackQueueLength to int");
            }
        }
        if (configMap.contains("minVelocityMagnitude")) {
            if (configMap["minVelocityMagnitude"].canConvert<double>())
                minVelocityMagnitude = configMap["minVelocityMagnitude"].toDouble();
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage("unable to convert minVelocityMagnitude to double");
            }
        }
        if (configMap.contains("headTailWeightVelocity")) {
            if (configMap["headTailWeightVelocity"].canConvert<double>())
                headTailWeightVelocity = configMap["headTailWeightVelocity"].toDouble();
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage("unable to convert headTailWeightVelocity to double");
            }
        }
        return rtnStatus;
    }

    RtnStatus FlyTrackConfig::setMiscFromMap(QVariantMap configMap) {
        RtnStatus rtnStatus;
        rtnStatus.success = true;
        rtnStatus.message = QString("");
        if (configMap.isEmpty())
        {
            rtnStatus.message = QString("flyTrack misc config empty");
            return rtnStatus;
        }
        if (configMap.contains("DEBUG")) {
            if (configMap["DEBUG"].canConvert<bool>())
                DEBUG = configMap["DEBUG"].toBool();
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage("unable to convert DEBUG to bool");
            }
        }
        if (configMap.contains("tmpOutDir")) {
            if (configMap["tmpOutDir"].canConvert<QString>())
                tmpOutDir = configMap["tmpOutDir"].toString();
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage("unable to convert tmpOutDir to string");
            }
        }
        if (configMap.contains("trackFileName")) {
            if (configMap["trackFileName"].canConvert<QString>())
                trackFileName = configMap["trackFileName"].toString();
            else {
                rtnStatus.success = false;
                rtnStatus.appendMessage("unable to convert trackFileName to string");
            }
        }
        return rtnStatus;
    }

    QVariantMap FlyTrackConfig::toMap()
    {
        fprintf(stderr, "FlyTrackConfig::toMap\n");
        // Create Device map
        QVariantMap configMap;
        QVariantMap bgEstMap;
        bgEstMap.insert("computeBgMode", computeBgMode);
        bgEstMap.insert("bgImageFilePath", bgImageFilePath);
        bgEstMap.insert("nFramesSkipBgEst", nFramesSkipBgEst);

        QVariantMap roiMap;
        QString roiTypeString;
        roiTypeToString(roiType, roiTypeString);
        roiMap.insert("roiType", roiTypeString);
        roiMap.insert("triangleCenterX", triangleCenterX);
        roiMap.insert("triangleCenterY", triangleCenterY);
        roiMap.insert("triangleSide", triangleSide);
        roiMap.insert("triangleAngle", triangleAngle);
        roiMap.insert("innerOffsetAlong", innerOffsetAlong);
        roiMap.insert("innerOffsetOut", innerOffsetOut);
        roiMap.insert("innerWidth", innerWidth);
        roiMap.insert("innerHeight", innerHeight);
        roiMap.insert("roiRightFeederCenterX", roiRightFeederCenterX);
        roiMap.insert("roiRightFeederCenterY", roiRightFeederCenterY);
        roiMap.insert("roiRightFeederWidth", roiRightFeederWidth);
        roiMap.insert("roiRightFeederHeight", roiRightFeederHeight);
		roiMap.insert("roiResetCenterX", roiResetCenterX);
		roiMap.insert("roiResetCenterY", roiResetCenterY);
		roiMap.insert("roiResetWidth", roiResetWidth);
		roiMap.insert("outerDepth", outerDepth);


        QVariantMap bgSubMap;
        bgSubMap.insert("backgroundThreshold", backgroundThreshold);
        QString flyVsBgModeString;
        flyVsBgModeToString(flyVsBgMode, flyVsBgModeString);
        bgSubMap.insert("flyVsBgMode", flyVsBgModeString);

        QVariantMap headTailMap;
        headTailMap.insert("historyBufferLength", historyBufferLength);
        headTailMap.insert("minVelocityMagnitude", minVelocityMagnitude);
        headTailMap.insert("headTailWeightVelocity", headTailWeightVelocity);

        QVariantMap miscMap;
        miscMap.insert("maxTrackQueueLength", maxTrackQueueLength);
        miscMap.insert("DEBUG", DEBUG);
        miscMap.insert("tmpOutDir", tmpOutDir);
        miscMap.insert("trackFileName", trackFileName);

        configMap.insert("bgEst", bgEstMap);
        configMap.insert("roi", roiMap);
        configMap.insert("bgSub", bgSubMap);
        configMap.insert("headTail", headTailMap);
        configMap.insert("misc", miscMap);

        fprintf(stderr, "Done with FlyTrackConfig::toMap\n");

        return configMap;
    }

    RtnStatus FlyTrackConfig::fromJson(QByteArray jsonConfigArray) {
        RtnStatus rtnStatus;
        rtnStatus.success = true;
        rtnStatus.message = QString("");

        bool ok;
        QVariantMap configMap = QtJson::parse(QString(jsonConfigArray), ok).toMap();
        if (!ok)
        {
            rtnStatus.success = false;
            rtnStatus.message = QString("FlyTrack unable to parse json configuration string");
            return rtnStatus;

        }
        rtnStatus = fromMap(configMap);
        return rtnStatus;
    }

    QByteArray FlyTrackConfig::toJson()
    {
        QVariantMap configMap = toMap();

        bool ok;
        QByteArray jsonConfigArray = QtJson::serialize(configMap, ok);
        if (!ok)
        {
            jsonConfigArray = QByteArray();
        }
        return jsonConfigArray;
    }

    // helper functions

    bool roiTypeToString(ROIType roiType, QString& roiTypeString) {
        switch (roiType) {
        case RECTANGLE:
            roiTypeString = QString("RECTANGLE");
            return true;
        default:
            return false;
        }
    }
    bool roiTypeFromString(QString roiTypeString, ROIType& roiType) {
        if (roiTypeString == "RECTANGLE") {
            roiType = RECTANGLE;
            return true;
        }
        roiTypeString = "UNKNOWN";
        return false;
    }

    bool flyVsBgModeToString(FlyVsBgModeType flyVsBgMode, QString& flyVsBgModeString) {
        switch (flyVsBgMode) {
        case FLY_DARKER_THAN_BG:
            flyVsBgModeString = QString("FLY_DARKER_THAN_BG");
            return true;
        case FLY_BRIGHTER_THAN_BG:
            flyVsBgModeString = QString("FLY_BRIGHTER_THAN_BG");
            return true;
        case FLY_ANY_DIFFERENCE_BG:
            flyVsBgModeString = QString("FLY_ANY_DIFFERENCE_BG");
            return true;
        default:
            return false;
        }
    }

    bool flyVsBgModeFromString(QString flyVsBgModeString, FlyVsBgModeType& flyVsBgMode) {
        if (flyVsBgModeString == "FLY_DARKER_THAN_BG") {
            flyVsBgMode = FLY_DARKER_THAN_BG;
            return true;
        }
        if (flyVsBgModeString == "FLY_BRIGHTER_THAN_BG") {
            flyVsBgMode = FLY_BRIGHTER_THAN_BG;
            return true;
        }
        if (flyVsBgModeString == "FLY_ANY_DIFFERENCE_BG") {
            flyVsBgMode = FLY_ANY_DIFFERENCE_BG;
            return true;
        }
        flyVsBgModeString = "UNKNOWN";
        return false;
    }

	void FlyTrackConfig::setRoiParams(ROIType roiTypeNew, double triangleCenterXNew, double triangleCenterYNew, double triangleSideNew, double triangleAngleNew, double innerOffsetAlongNew, double innerOffsetOutNew, double innerWidthNew, double innerHeightNew, double roiRightFeederCenterXNew, double roiRightFeederCenterYNew, double roiRightFeederWidthNew, double roiRightFeederHeightNew, double roiResetCenterXNew, double roiResetCenterYNew, double roiResetWidthNew, double resetResetHeightNew) {
        roiType = roiTypeNew;
        triangleCenterX = triangleCenterXNew;
        triangleCenterY = triangleCenterYNew;
        triangleSide = triangleSideNew;
        triangleAngle = triangleAngleNew;
		innerOffsetAlong = innerOffsetAlongNew;
		innerOffsetOut = innerOffsetOutNew;
		innerWidth = innerWidthNew;
		innerHeight = innerHeightNew;
		roiRightFeederCenterX = roiRightFeederCenterXNew;
		roiRightFeederCenterY = roiRightFeederCenterYNew;
		roiRightFeederWidth = roiRightFeederWidthNew;
		roiRightFeederHeight = roiRightFeederHeightNew;
		roiResetCenterX = roiResetCenterXNew;
		roiResetCenterY = roiResetCenterYNew;
		roiResetWidth = roiResetWidthNew;
		outerDepth = resetResetHeightNew;
        

    }

}