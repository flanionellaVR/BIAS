#ifdef WITH_ARENA
#include "camera_device_arena.hpp"
#include "utils_arena.hpp"
#include "exception.hpp"
#include <sstream>
#include <iostream>
#include <algorithm>
#include "ArenaCAPI.h"


namespace bias {

    const std::string CameraDevice_arena::ExternalTriggerSource = std::string("Line0");

    static std::string arenaErrorString(const char *function, const std::string &msg, AC_ERROR err)
    {
        std::stringstream ssError;
        ssError << function << ": " << msg << ", error = " << err;
        ssError << " (" << getLastErrorMessage_arena() << ")";
        return ssError.str();
    }


    CameraDevice_arena::CameraDevice_arena() : CameraDevice() {}


    CameraDevice_arena::CameraDevice_arena(Guid guid) : CameraDevice(guid)
    {
        hSystem_ = acquireSystem_arena();
    }


    CameraDevice_arena::~CameraDevice_arena()
    {
        // Destructors must not let exceptions escape (it is implicitly
        // noexcept, so doing so would call std::terminate). stopCapture(),
        // disconnect() and the release call below can all throw RuntimeError
        // on failure, so any such error is caught and logged here instead of
        // being propagated.
        try
        {
            if (capturing_)
            {
                stopCapture();
            }

            if (connected_)
            {
                disconnect();
            }

            if (hSystem_ != nullptr)
            {
                hSystem_ = nullptr;
                releaseSystem_arena();
            }
        }
        catch (std::exception &e)
        {
            std::cerr << __FUNCTION__ << ": error during camera shutdown: " << e.what() << std::endl;
        }
    }


    CameraLib CameraDevice_arena::getCameraLib()
    {
        return guid_.getCameraLib();
    }


    void CameraDevice_arena::connect()
    {
        if (!connected_)
        {
            hDevice_ = createDevice_arena(hSystem_, guid_.toString());

            AC_ERROR err = acDeviceGetNodeMap(hDevice_, &hNodeMapCamera_);
            if ((err != AC_ERR_SUCCESS) || (hNodeMapCamera_ == nullptr))
            {
                std::string errMsg = arenaErrorString(__FUNCTION__, "unable to get Arena device node map", err);
                destroyDevice_arena(hSystem_, hDevice_);
                hDevice_ = nullptr;
                throw RuntimeError(ERROR_ARENA_GET_NODE_MAP, errMsg);
            }

            err = acDeviceGetTLStreamNodeMap(hDevice_, &hNodeMapTLStream_);
            if ((err != AC_ERR_SUCCESS) || (hNodeMapTLStream_ == nullptr))
            {
                std::string errMsg = arenaErrorString(__FUNCTION__, "unable to get Arena stream node map", err);
                destroyDevice_arena(hSystem_, hDevice_);
                hDevice_ = nullptr;
                throw RuntimeError(ERROR_ARENA_GET_NODE_MAP, errMsg);
            }

            connected_ = true;

            // Get camera info
            Node_arena vendorNameNode = cameraNode("DeviceVendorName");
            if (vendorNameNode.isReadable())
            {
                vendorName_ = vendorNameNode.stringValue();
            }
            Node_arena modelNameNode = cameraNode("DeviceModelName");
            if (modelNameNode.isReadable())
            {
                modelName_ = modelNameNode.stringValue();
            }
            Node_arena serialNumberNode = cameraNode("DeviceSerialNumber");
            if (serialNumberNode.isReadable())
            {
                serialNumber_ = serialNumberNode.stringValue();
            }
            printInfo();

            // Default settings
            // ----------------------------------------------------------------------------------------------

            // Exposure defaults
            Node_arena exposureAutoNode = cameraNode("ExposureAuto");
            if (exposureAutoNode.isAvailable() && exposureAutoNode.isWritable())
            {
                exposureAutoNode.setSymbolic("Off");
            }

            // Gain defaults
            Node_arena gainAutoNode = cameraNode("GainAuto");
            if (gainAutoNode.isAvailable() && gainAutoNode.isWritable())
            {
                gainAutoNode.setSymbolic("Off");
            }

            // Trigger defaults
            Node_arena triggerSelectorNode = cameraNode("TriggerSelector");
            if (triggerSelectorNode.isAvailable() && triggerSelectorNode.isWritable())
            {
                triggerSelectorNode.setSymbolic("FrameStart");
            }

            triggerType_ = getTriggerType();
            setTriggerInternal();

            // Framerate defaults
            Node_arena frameRateEnableNode = cameraNode("AcquisitionFrameRateEnable");
            if (frameRateEnableNode.isAvailable() && frameRateEnableNode.isWritable())
            {
                frameRateEnableNode.setBoolValue(true);
            }

            setupStream();
        }
    }


    void CameraDevice_arena::disconnect()
    {
        if (capturing_)
        {
            stopCapture();
        }

        if (connected_)
        {
            void *hDevice = hDevice_;
            hDevice_ = nullptr;
            hNodeMapCamera_ = nullptr;
            hNodeMapTLStream_ = nullptr;
            connected_ = false;
            destroyDevice_arena(hSystem_, hDevice);
        }
    }


    void CameraDevice_arena::startCapture()
    {
        if (!connected_)
        {
            std::stringstream ssError;
            ssError << __FUNCTION__;
            ssError << ": unable to start Arena capture - not connected";
            throw RuntimeError(ERROR_ARENA_START_CAPTURE, ssError.str());
        }

        if (!capturing_)
        {
            // Set acquisition mode
            Node_arena acqModeNode = cameraNode("AcquisitionMode");
            if (acqModeNode.isAvailable() && acqModeNode.isWritable())
            {
                acqModeNode.setSymbolic("Continuous");
            }

            // The pixel format can't change while streaming - remember its name, it
            // is used to pick the conversion applied to grabbed images.
            capturePixelFormatName_ = std::string("");
            Node_arena pixelFormatNode = cameraNode("PixelFormat");
            if (pixelFormatNode.isReadable())
            {
                capturePixelFormatName_ = pixelFormatNode.symbolic();
            }

            // Begin acquisition
            AC_ERROR err = acDeviceStartStream(hDevice_);
            if (err != AC_ERR_SUCCESS)
            {
                throw RuntimeError(ERROR_ARENA_START_CAPTURE, arenaErrorString(__FUNCTION__, "unable to start Arena stream", err));
            }

            capturing_ = true;
        }
    }


    void CameraDevice_arena::stopCapture()
    {
        if (capturing_)
        {
            capturing_ = false;
            AC_ERROR err = acDeviceStopStream(hDevice_);
            if (err != AC_ERR_SUCCESS)
            {
                throw RuntimeError(ERROR_ARENA_STOP_CAPTURE, arenaErrorString(__FUNCTION__, "unable to stop Arena stream", err));
            }
        }
    }


    cv::Mat CameraDevice_arena::grabImage()
    {
        cv::Mat image;
        grabImage(image);
        return image;
    }


    void CameraDevice_arena::grabImage(cv::Mat &image)
    {
        // Note: like the other backends this returns an empty image, rather
        // than raising an error, when no (complete) frame is available.

        if (!capturing_)
        {
            image.release();
            return;
        }

        acBuffer hBuffer = nullptr;
        AC_ERROR err = acDeviceGetBuffer(hDevice_, GrabTimeoutMs, &hBuffer);
        if (err == AC_ERR_TIMEOUT)
        {
            image.release();
            return;
        }
        if ((err != AC_ERR_SUCCESS) || (hBuffer == nullptr))
        {
            image.release();
            throw RuntimeError(ERROR_ARENA_GRAB_IMAGE, arenaErrorString(__FUNCTION__, "unable to get next image", err));
        }

        acBuffer hBufferConv = nullptr;
        try
        {
            bool8_t isIncomplete = false;
            err = acBufferIsIncomplete(hBuffer, &isIncomplete);
            if (err != AC_ERR_SUCCESS)
            {
                throw RuntimeError(ERROR_ARENA_GRAB_IMAGE, arenaErrorString(__FUNCTION__, "unable to determine if image is complete", err));
            }

            bool8_t hasImageData = true;
            err = acBufferHasImageData(hBuffer, &hasImageData);
            if (err != AC_ERR_SUCCESS)
            {
                hasImageData = true;
            }

            if (isIncomplete || !hasImageData)
            {
                // Typically lost packets on the network link.
                acDeviceRequeueBuffer(hDevice_, hBuffer);
                image.release();
                return;
            }

            // Timestamp (camera clock)
            uint64_t timeStamp_ns = 0;
            err = acImageGetTimestampNs(hBuffer, &timeStamp_ns);
            if (err != AC_ERR_SUCCESS)
            {
                throw RuntimeError(ERROR_ARENA_IMAGE_INFO, arenaErrorString(__FUNCTION__, "unable to get image timestamp", err));
            }

            uint64_t pixelFormat = 0;
            err = acImageGetPixelFormat(hBuffer, &pixelFormat);
            if (err != AC_ERR_SUCCESS)
            {
                throw RuntimeError(ERROR_ARENA_IMAGE_INFO, arenaErrorString(__FUNCTION__, "unable to get image pixel format", err));
            }

            // Convert to a format OpenCV can use, if required
            acBuffer hBufferSrc = hBuffer;
            uint64_t convPixelFormat = getSuitablePixelFormat_arena(pixelFormat, capturePixelFormatName_);
            if (convPixelFormat != pixelFormat)
            {
                err = acImageFactoryConvert(hBuffer, convPixelFormat, &hBufferConv);
                if ((err != AC_ERR_SUCCESS) || (hBufferConv == nullptr))
                {
                    hBufferConv = nullptr;
                    throw RuntimeError(ERROR_ARENA_IMAGE_CONVERT, arenaErrorString(__FUNCTION__, "unable to convert image", err));
                }
                hBufferSrc = hBufferConv;
            }

            size_t width = 0;
            size_t height = 0;
            size_t paddingX = 0;
            uint8_t *dataPtr = nullptr;
            AC_ERROR errWidth = acImageGetWidth(hBufferSrc, &width);
            AC_ERROR errHeight = acImageGetHeight(hBufferSrc, &height);
            AC_ERROR errData = acImageGetData(hBufferSrc, &dataPtr);
            if (acImageGetPaddingX(hBufferSrc, &paddingX) != AC_ERR_SUCCESS)
            {
                paddingX = 0;
            }
            if ((errWidth != AC_ERR_SUCCESS) || (errHeight != AC_ERR_SUCCESS) || (errData != AC_ERR_SUCCESS) ||
                    (dataPtr == nullptr) || (width == 0) || (height == 0))
            {
                throw RuntimeError(ERROR_ARENA_IMAGE_INFO, arenaErrorString(__FUNCTION__, "unable to get image size or data", errData));
            }

            int opencvPixelFormat = getCompatibleOpencvFormat_arena(convPixelFormat);
            size_t stride = width*CV_ELEM_SIZE(opencvPixelFormat) + paddingX;

            cv::Mat imageTmp = cv::Mat(int(height), int(width), opencvPixelFormat, dataPtr, stride);
            imageTmp.copyTo(image);

            timeStamp_ns_ = timeStamp_ns;
            uint64_t seconds = timeStamp_ns_/UINT64_C(1000000000);
            uint64_t microSeconds = timeStamp_ns_/UINT64_C(1000) - UINT64_C(1000000)*seconds;
            timeStamp_.seconds = (unsigned long long)(seconds);
            timeStamp_.microSeconds = (unsigned int)(microSeconds);
        }
        catch (...)
        {
            if (hBufferConv != nullptr)
            {
                acImageFactoryDestroy(hBufferConv);
            }
            acDeviceRequeueBuffer(hDevice_, hBuffer);
            image.release();
            throw;
        }

        if (hBufferConv != nullptr)
        {
            acImageFactoryDestroy(hBufferConv);
        }

        err = acDeviceRequeueBuffer(hDevice_, hBuffer);
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_GRAB_IMAGE, arenaErrorString(__FUNCTION__, "unable to requeue image buffer", err));
        }
    }


    bool CameraDevice_arena::isColor()
    {
        Node_arena pixelFormatNode = cameraNode("PixelFormat");
        for (auto &entry : pixelFormatNode.entries())
        {
            if (isColorPixelFormatName_arena(entry.first))
            {
                return true;
            }
        }
        return false;
    }


    VideoMode CameraDevice_arena::getVideoMode()
    {
        VideoModeList allowedVideoModes = getAllowedVideoModes();
        if (allowedVideoModes.size() != 1)
        {
            std::stringstream ssError;
            ssError << __FUNCTION__;
            ssError << ": more than one video mode supported (DEVEL)";
            throw RuntimeError(ERROR_ARENA_VIDEOMODE_SUPPORT, ssError.str());
        }
        return allowedVideoModes.front();
    }


    FrameRate CameraDevice_arena::getFrameRate()
    {
        VideoMode videoMode = getVideoMode();
        FrameRateList allowedFrameRates = getAllowedFrameRates(videoMode);
        if (allowedFrameRates.size() != 1)
        {
            std::stringstream ssError;
            ssError << __FUNCTION__;
            ssError << ": more than one framerate supported (DEVEL)";
            throw RuntimeError(ERROR_ARENA_FRAMERATE_SUPPORT, ssError.str());
        }
        return allowedFrameRates.front();
    }


    ImageMode CameraDevice_arena::getImageMode()
    {
        ImageModeList allowedModeList = getAllowedImageModes();
        if (allowedModeList.size() != 1)
        {
            std::stringstream ssError;
            ssError << __FUNCTION__;
            ssError << ": more than one imagemode supported (DEVEL)";
            throw RuntimeError(ERROR_ARENA_IMAGEMODE_SUPPORT, ssError.str());
        }
        return allowedModeList.front();
    }


    VideoModeList CameraDevice_arena::getAllowedVideoModes()
    {
        // Note:
        // --------------------------------------------------------------------
        // Like Spinnaker, the Arena SDK doesn't have the concept of VideoModes
        // found in FlyCapture2 and libdc1394 so we fake it.
        // --------------------------------------------------------------------
        VideoModeList allowedVideoModes = {VIDEOMODE_FORMAT7};
        return allowedVideoModes;
    }


    FrameRateList CameraDevice_arena::getAllowedFrameRates(VideoMode vidMode)
    {
        FrameRateList allowedFrameRates = {};
        if (vidMode == VIDEOMODE_FORMAT7)
        {
            allowedFrameRates.push_back(FRAMERATE_FORMAT7);
        }
        return allowedFrameRates;
    }


    ImageModeList CameraDevice_arena::getAllowedImageModes()
    {
        // Note: only IMAGEMODE_0 is supported (see Spinnaker backend).
        ImageModeList allImageModes = {IMAGEMODE_0};
        return allImageModes;
    }


    PropertyInfo CameraDevice_arena::getPropertyInfo(PropertyType propType)
    {
        PropertyInfo propInfo;
        propInfo.type = propType;

        if (isArenaSupportedPropertyType(propType))
        {
            if (getPropertyInfoDispatchMap_.count(propType) > 0)
            {
                propInfo = getPropertyInfoDispatchMap_[propType](this);
            }
        }

        return propInfo;
    }


    Property CameraDevice_arena::getProperty(PropertyType propType)
    {
        Property prop;
        prop.type = propType;

        if (isArenaSupportedPropertyType(propType))
        {
            if (getPropertyDispatchMap_.count(propType) > 0)
            {
                prop = getPropertyDispatchMap_[propType](this);
            }
        }

        return prop;
    }


    void CameraDevice_arena::setProperty(Property prop)
    {
        std::string settableMsg("");
        bool isSettable = isPropertySettable(prop.type, settableMsg);

        if (!isSettable)
        {
            throw RuntimeError(ERROR_ARENA_PROPERTY_NOT_SETTABLE, settableMsg);
        }

        setPropertyDispatchMap_[prop.type](this,prop);
    }


    bool CameraDevice_arena::isPropertySettable(PropertyType propType, std::string &msg)
    {
        if (!isArenaSupportedPropertyType(propType))
        {
            msg = std::string("PropertyType is not supported by Arena Backend");
            return false;
        }

        if (setPropertyDispatchMap_.count(propType) <= 0)
        {
            msg = std::string("PropertyType is not is setter dispatch map");
            return false;
        }

        PropertyInfo propInfo = getPropertyInfo(propType);
        if (!propInfo.present)
        {
            msg = std::string("PropertyType is not present");
            return false;
        }
        return true;
    }


    Format7Settings CameraDevice_arena::getFormat7Settings()
    {
        Format7Settings settings;

        settings.mode = getImageMode();
        settings.offsetX = (unsigned int)(cameraNode("OffsetX").value());
        settings.offsetY = (unsigned int)(cameraNode("OffsetY").value());
        settings.width = (unsigned int)(cameraNode("Width").value());
        settings.height = (unsigned int)(cameraNode("Height").value());
        settings.pixelFormat = getPixelFormat();

        return settings;
    }


    Format7Info CameraDevice_arena::getFormat7Info(ImageMode imgMode)
    {
        Format7Info format7Info;

        format7Info.mode = imgMode;
        format7Info.supported = isSupported(imgMode);

        if (format7Info.supported)
        {
            Node_arena offsetXNode = cameraNode("OffsetX");
            Node_arena offsetYNode = cameraNode("OffsetY");
            Node_arena widthNode = cameraNode("Width");
            Node_arena heightNode = cameraNode("Height");

            // The maximum of the Width (Height) node shrinks as OffsetX (OffsetY)
            // grows, so the full size is the node's maximum plus the current offset.
            format7Info.maxWidth = (unsigned int)(widthNode.maxValue() + offsetXNode.value());
            format7Info.maxHeight = (unsigned int)(heightNode.maxValue() + offsetYNode.value());

            // ... unless the camera reports the full size directly.
            Node_arena widthMaxNode = cameraNode("WidthMax");
            if (widthMaxNode.isReadable())
            {
                format7Info.maxWidth = (unsigned int)(widthMaxNode.value());
            }
            Node_arena heightMaxNode = cameraNode("HeightMax");
            if (heightMaxNode.isReadable())
            {
                format7Info.maxHeight = (unsigned int)(heightMaxNode.value());
            }

            format7Info.imageHStepSize = std::max((unsigned int)(widthNode.increment()), 1u);
            format7Info.imageVStepSize = std::max((unsigned int)(heightNode.increment()), 1u);
            format7Info.offsetHStepSize = std::max((unsigned int)(offsetXNode.increment()), 1u);
            format7Info.offsetVStepSize = std::max((unsigned int)(offsetYNode.increment()), 1u);
        }

        return format7Info;
    }


    bool CameraDevice_arena::validateFormat7Settings(Format7Settings settings)
    {
        bool ok = true;

        if (!isSupported(settings.mode))
        {
            return false;
        }

        Format7Info info = getFormat7Info(settings.mode);

        if ((settings.width == 0) || (settings.height == 0))
        {
            ok = false;
        }
        if ((settings.width + settings.offsetX) > info.maxWidth)
        {
            ok = false;
        }
        if ((settings.height + settings.offsetY) > info.maxHeight)
        {
            ok = false;
        }
        if (settings.width%info.imageHStepSize != 0)
        {
            ok = false;
        }
        if (settings.height%info.imageVStepSize != 0)
        {
            ok = false;
        }
        if (settings.offsetX%info.offsetHStepSize != 0)
        {
            ok = false;
        }
        if (settings.offsetY%info.offsetVStepSize != 0)
        {
            ok = false;
        }
        if (!isSupportedPixelFormat(settings.pixelFormat, settings.mode))
        {
            ok = false;
        }

        return ok;
    }


    void CameraDevice_arena::setFormat7Configuration(Format7Settings settings, float percentSpeed)
    {
        bool ok = validateFormat7Settings(settings);
        if (!ok)
        {
            return;
        }

        Node_arena pixelFormatNode = cameraNode("PixelFormat");
        pixelFormatNode.setEnumIntValue(int64_t(convertPixelFormat_to_arena(settings.pixelFormat)));

        Node_arena offsetXNode = cameraNode("OffsetX");
        Node_arena offsetYNode = cameraNode("OffsetY");
        Node_arena widthNode = cameraNode("Width");
        Node_arena heightNode = cameraNode("Height");

        // The offsets limit the allowed size (and vice versa), so move the ROI to
        // the origin first, then set its size and finally move it into place.
        offsetXNode.setValue(0.0);
        offsetYNode.setValue(0.0);
        widthNode.setValue(double(settings.width));
        heightNode.setValue(double(settings.height));
        offsetXNode.setValue(double(settings.offsetX));
        offsetYNode.setValue(double(settings.offsetY));
    }


    PixelFormatList CameraDevice_arena::getListOfSupportedPixelFormats(ImageMode imgMode)
    {
        PixelFormatList pixelFormatList;
        if (isSupported(imgMode))
        {
            Node_arena pixelFormatNode = cameraNode("PixelFormat");
            for (auto &entry : pixelFormatNode.entries())
            {
                uint64_t format_arena = uint64_t(entry.second);
                if (isAllowedPixelFormat_arena(format_arena))
                {
                    PixelFormat format = convertPixelFormat_from_arena(format_arena);
                    if (std::find(pixelFormatList.begin(), pixelFormatList.end(), format) == pixelFormatList.end())
                    {
                        pixelFormatList.push_back(format);
                    }
                }
            }
        }
        return pixelFormatList;
    }


    bool CameraDevice_arena::isSupported(VideoMode vidMode, FrameRate frmRate)
    {
        VideoModeList allowedVideoModes = getAllowedVideoModes();
        FrameRateList allowedFrameRates = getAllowedFrameRates(vidMode);
        bool videoModeFound = (std::find(allowedVideoModes.begin(), allowedVideoModes.end(), vidMode) != allowedVideoModes.end());
        bool frameRateFound = (std::find(allowedFrameRates.begin(), allowedFrameRates.end(), frmRate) != allowedFrameRates.end());
        return (videoModeFound && frameRateFound);
    }


    bool CameraDevice_arena::isSupported(ImageMode imgMode)
    {
        ImageModeList allowedModes = getAllowedImageModes();
        bool found = (std::find(allowedModes.begin(), allowedModes.end(), imgMode) != allowedModes.end());
        return found;
    }


    unsigned int CameraDevice_arena::getNumberOfImageMode()
    {
        return (unsigned int)(getAllowedImageModes().size());
    }


    void CameraDevice_arena::setTriggerInternal()
    {
        Node_arena triggerModeNode = cameraNode("TriggerMode");
        if (triggerModeNode.isAvailable() && triggerModeNode.isWritable())
        {
            triggerModeNode.setSymbolic("Off");
            triggerType_ = TRIGGER_INTERNAL;
        }
        else
        {
            std::stringstream ssError;
            ssError << __FUNCTION__;
            ssError << ": triggerModeNode is not available or writable";
            throw RuntimeError(ERROR_ARENA_SET_TRIGGER_INTERNAL, ssError.str());
        }
    }


    void CameraDevice_arena::setTriggerExternal()
    {
        Node_arena triggerModeNode = cameraNode("TriggerMode");
        if (triggerModeNode.isAvailable() && triggerModeNode.isWritable())
        {
            triggerModeNode.setSymbolic("Off");
        }
        else
        {
            std::stringstream ssError;
            ssError << __FUNCTION__;
            ssError << ": triggerModeNode is not available or writable";
            throw RuntimeError(ERROR_ARENA_SET_TRIGGER_EXTERNAL, ssError.str());
        }

        Node_arena triggerSelectorNode = cameraNode("TriggerSelector");
        if (triggerSelectorNode.isAvailable() && triggerSelectorNode.isWritable())
        {
            triggerSelectorNode.setSymbolic("FrameStart");
        }

        Node_arena triggerSourceNode = cameraNode("TriggerSource");
        if (triggerSourceNode.isAvailable() && triggerSourceNode.isWritable())
        {
            triggerSourceNode.setSymbolic(ExternalTriggerSource);
        }
        else
        {
            std::stringstream ssError;
            ssError << __FUNCTION__;
            ssError << ": triggerSourceNode is not available or writable";
            throw RuntimeError(ERROR_ARENA_SET_TRIGGER_EXTERNAL, ssError.str());
        }

        Node_arena triggerActivationNode = cameraNode("TriggerActivation");
        if (triggerActivationNode.isAvailable() && triggerActivationNode.isWritable())
        {
            triggerActivationNode.setSymbolic("RisingEdge");
        }
        else
        {
            std::stringstream ssError;
            ssError << __FUNCTION__;
            ssError << ": triggerActivationNode is not available or writable";
            throw RuntimeError(ERROR_ARENA_SET_TRIGGER_EXTERNAL, ssError.str());
        }

        triggerModeNode.setSymbolic("On");
        triggerType_ = TRIGGER_EXTERNAL;
    }


    TriggerType CameraDevice_arena::getTriggerType()
    {
        Node_arena triggerModeNode = cameraNode("TriggerMode");
        if (!(triggerModeNode.isAvailable() && triggerModeNode.isReadable()))
        {
            std::stringstream ssError;
            ssError << __FUNCTION__;
            ssError << ": triggerModeNode is not available or readable";
            throw RuntimeError(ERROR_ARENA_GET_TRIGGER_TYPE, ssError.str());
        }
        std::string modeSymb = triggerModeNode.symbolic();
        return (modeSymb == std::string("On")) ? TRIGGER_EXTERNAL : TRIGGER_INTERNAL;
    }


    TimeStamp CameraDevice_arena::getImageTimeStamp()
    {
        return timeStamp_;
    }


    std::string CameraDevice_arena::getVendorName()
    {
        return vendorName_;
    }


    std::string CameraDevice_arena::getModelName()
    {
        return modelName_;
    }


    std::string CameraDevice_arena::toString()
    {
        std::stringstream ss;

        ss << std::endl;
        ss << "------------------ " << std::endl;
        ss << "Camera Information " << std::endl;
        ss << "------------------ " << std::endl;
        ss << std::endl;

        ss << " guid:           " << guid_.toString() << std::endl;
        ss << " serial number:  " << serialNumber_ << std::endl;
        ss << " camera vendor:  " << vendorName_ << std::endl;
        ss << " camera model:   " << modelName_ << std::endl;
        ss << std::endl;

        return ss.str();
    }


    void CameraDevice_arena::printGuid()
    {
        guid_.printValue();
    }


    void CameraDevice_arena::printInfo()
    {
        std::cout << toString();
    }


    std::string CameraDevice_arena::nodeMapToString()
    {
        std::stringstream ss;
        if (!connected_)
        {
            return ss.str();
        }

        uint64_t numNodes = 0;
        AC_ERROR err = acNodeMapGetNumNodes(hNodeMapCamera_, &numNodes);
        if (err != AC_ERR_SUCCESS)
        {
            return ss.str();
        }

        static const char *accessModeNames[] = {"NI", "NA", "WO", "RO", "RW", "??"};

        for (uint64_t i=0; i<numNodes; i++)
        {
            acNode hNode = nullptr;
            AC_ACCESS_MODE mode = AC_ACCESS_MODE_NI;
            err = acNodeMapGetNodeByIndexAndAccessMode(hNodeMapCamera_, size_t(i), &hNode, &mode);
            if ((err != AC_ERR_SUCCESS) || (hNode == nullptr))
            {
                continue;
            }

            bool8_t isFeature = false;
            if ((acNodeIsFeature(hNode, &isFeature) != AC_ERR_SUCCESS) || !isFeature)
            {
                continue;
            }

            AC_INTERFACE_TYPE type = -1;
            acNodeGetPrincipalInterfaceType(hNode, &type);
            if ((type == AC_INTERFACE_TYPE_CATEGORY) || (type == AC_INTERFACE_TYPE_REGISTER) ||
                (type == AC_INTERFACE_TYPE_PORT) || (type == AC_INTERFACE_TYPE_ENUMENTRY) ||
                (type == AC_INTERFACE_TYPE_COMMAND))
            {
                continue;
            }

            char nameBuf[512];
            size_t nameLen = sizeof(nameBuf);
            if (acNodeGetName(hNode, nameBuf, &nameLen) != AC_ERR_SUCCESS)
            {
                continue;
            }

            int modeIndex = ((mode >= 0) && (mode <= 4)) ? int(mode) : 5;
            ss << nameBuf << " [" << accessModeNames[modeIndex] << "]";

            if ((mode == AC_ACCESS_MODE_RO) || (mode == AC_ACCESS_MODE_RW))
            {
                char valueBuf[1024];
                size_t valueLen = sizeof(valueBuf);
                if (acValueToString(hNode, valueBuf, &valueLen) == AC_ERR_SUCCESS)
                {
                    ss << " = " << valueBuf;
                }
                if ((type == AC_INTERFACE_TYPE_INTEGER) || (type == AC_INTERFACE_TYPE_FLOAT))
                {
                    Node_arena node(hNodeMapCamera_, std::string(nameBuf));
                    try
                    {
                        ss << "  (min " << node.minValue() << ", max " << node.maxValue();
                        ss << ", inc " << node.increment() << ", unit '" << node.unit() << "')";
                    }
                    catch (RuntimeError&) {}
                }
                if (type == AC_INTERFACE_TYPE_ENUMERATION)
                {
                    Node_arena node(hNodeMapCamera_, std::string(nameBuf));
                    try
                    {
                        ss << "  {";
                        bool first = true;
                        for (auto &entry : node.entries())
                        {
                            ss << (first ? "" : ", ") << entry.first;
                            first = false;
                        }
                        ss << "}";
                    }
                    catch (RuntimeError&) {}
                }
            }
            ss << std::endl;
        }
        return ss.str();
    }


    // Private methods
    // -------------------------------------------------------------------------

    Node_arena CameraDevice_arena::cameraNode(const std::string &name)
    {
        return Node_arena(hNodeMapCamera_, name);
    }


    Node_arena CameraDevice_arena::streamNode(const std::string &name)
    {
        return Node_arena(hNodeMapTLStream_, name);
    }


    void CameraDevice_arena::setupStream()
    {
        // Settings recommended by LUCID for GigE Vision streams. They are not
        // present for other transport layers, hence the availability checks.

        // Negotiate the largest packet size the network link supports
        Node_arena autoPacketSizeNode = streamNode("StreamAutoNegotiatePacketSize");
        if (autoPacketSizeNode.isAvailable() && autoPacketSizeNode.isWritable())
        {
            autoPacketSizeNode.setBoolValue(true);
        }

        // Ask the camera to resend packets lost on the way
        Node_arena packetResendNode = streamNode("StreamPacketResendEnable");
        if (packetResendNode.isAvailable() && packetResendNode.isWritable())
        {
            packetResendNode.setBoolValue(true);
        }
    }


    // Property helpers
    // ------------------------------------------------------------------------

    void CameraDevice_arena::getShutterLimits(Node_arena &exposureTimeNode, double &lowerLimit, double &upperLimit)
    {
        // The range the shutter can be set to from the GUI: the camera's current 
        // exposure time range (which depends on the frame rate) cut down to the 
        // artificial limits which keep the GUI usable. The same range is used for 
        // the absolute values and for the integer (slider) values so that the 
        // two always agree.
        lowerLimit = std::max(exposureTimeNode.minValue(), MinAllowedShutterUs);
        upperLimit = std::min(exposureTimeNode.maxValue(), MaxAllowedShutterUs);
        if (upperLimit < lowerLimit)
        {
            lowerLimit = exposureTimeNode.minValue();
            upperLimit = exposureTimeNode.maxValue();
        }
    }


    PropertyInfo CameraDevice_arena::getNumberPropertyInfo(PropertyType propType, const std::string &nodeName, bool manualCapable)
    {
        Node_arena node = cameraNode(nodeName);

        PropertyInfo propInfo;
        propInfo.type = propType;
        propInfo.present = node.isAvailable();

        if (propInfo.present)
        {
            propInfo.autoCapable = false;
            propInfo.manualCapable = manualCapable;
            propInfo.absoluteCapable = true;
            propInfo.onePushCapable = false;
            propInfo.onOffCapable = false;
            propInfo.readOutCapable = false;
            if (node.isReadable())
            {
                propInfo.minValue = node.minIntValue();
                propInfo.maxValue = node.maxIntValue();
                propInfo.minAbsoluteValue = float(node.minValue());
                propInfo.maxAbsoluteValue = float(node.maxValue());
                propInfo.haveUnits = !node.unit().empty();
                propInfo.units = node.unit();
                propInfo.unitsAbbr = node.unit();
            }
        }

        return propInfo;
    }


    Property CameraDevice_arena::getNumberProperty(PropertyType propType, const std::string &nodeName)
    {
        Node_arena node = cameraNode(nodeName);

        Property prop;
        prop.type = propType;
        prop.present = node.isAvailable();

        if (prop.present)
        {
            prop.absoluteControl = true;
            prop.onePush = false;
            prop.on = true;
            prop.autoActive = false;
            prop.valueA = 0;
            prop.valueB = 0;
            if (node.isReadable())
            {
                prop.value = node.intValue();
                prop.absoluteValue = float(node.value());
            }
        }

        return prop;
    }


    void CameraDevice_arena::setNumberProperty(Property prop, const std::string &nodeName)
    {
        Node_arena node = cameraNode(nodeName);
        if (node.isAvailable() && node.isReadable() && node.isWritable())
        {
            if (prop.absoluteControl)
            {
                node.setValue(prop.absoluteValue);
            }
            else
            {
                node.setValueFromInt(prop.value);
            }
        }
    }


    void CameraDevice_arena::setAutoNumberProperty(Property prop, const std::string &autoNodeName, const std::string &nodeName, bool useShutterLimits)
    {
        Node_arena autoNode = cameraNode(autoNodeName);
        Node_arena node = cameraNode(nodeName);

        if (autoNode.isAvailable() && autoNode.isWritable())
        {
            if (prop.onePush && autoNode.hasSymbolic("Once"))
            {
                autoNode.setSymbolic("Once");
                return;
            }

            if (prop.autoActive && autoNode.hasSymbolic("Continuous"))
            {
                autoNode.setSymbolic("Continuous");
                return;
            }
            else
            {
                autoNode.setSymbolic("Off");
            }
        }

        if (node.isAvailable() && node.isReadable() && node.isWritable())
        {
            if (prop.absoluteControl)
            {
                node.setValue(prop.absoluteValue);
            }
            else if (useShutterLimits)
            {
                double lowerLimit = 0.0;
                double upperLimit = 0.0;
                getShutterLimits(node, lowerLimit, upperLimit);
                node.setValueFromIntWithLimits(prop.value, lowerLimit, upperLimit);
            }
            else
            {
                node.setValueFromInt(prop.value);
            }
        }
    }


    // Get PropertyInfo methods
    // --------------------------

    PropertyInfo CameraDevice_arena::getPropertyInfoBrightness()
    {
        return getNumberPropertyInfo(PROPERTY_TYPE_BRIGHTNESS, "BlackLevel", true);
    }


    PropertyInfo CameraDevice_arena::getPropertyInfoGamma()
    {
        Node_arena gammaNode = cameraNode("Gamma");
        Node_arena gammaEnableNode = cameraNode("GammaEnable");

        PropertyInfo propInfo;
        propInfo.type = PROPERTY_TYPE_GAMMA;
        propInfo.present = gammaEnableNode.isAvailable() || gammaNode.isAvailable();

        if (propInfo.present)
        {
            propInfo.autoCapable = false;
            propInfo.manualCapable = true;
            propInfo.absoluteCapable = true;
            propInfo.onePushCapable = false;
            propInfo.onOffCapable = gammaEnableNode.isAvailable() && gammaEnableNode.isWritable();
            propInfo.readOutCapable = false;
            if (gammaNode.isAvailable() && gammaNode.isReadable())
            {
                propInfo.minValue = gammaNode.minIntValue();
                propInfo.maxValue = gammaNode.maxIntValue();
                propInfo.minAbsoluteValue = float(gammaNode.minValue());
                propInfo.maxAbsoluteValue = float(gammaNode.maxValue());
                propInfo.haveUnits = !gammaNode.unit().empty();
                propInfo.units =  gammaNode.unit();
                propInfo.unitsAbbr = gammaNode.unit();
            }
        }

        return propInfo;
    }


    PropertyInfo CameraDevice_arena::getPropertyInfoShutter()
    {
        Node_arena exposureAutoNode = cameraNode("ExposureAuto");
        Node_arena exposureTimeNode = cameraNode("ExposureTime");

        PropertyInfo propInfo;
        propInfo.type = PROPERTY_TYPE_SHUTTER;
        propInfo.present = exposureAutoNode.isAvailable() || exposureTimeNode.isAvailable();

        if (propInfo.present)
        {
            propInfo.autoCapable = false;
            propInfo.manualCapable = true;
            propInfo.onePushCapable = false;
            if (exposureAutoNode.isAvailable() && exposureAutoNode.isReadable())
            {
                propInfo.autoCapable = exposureAutoNode.hasSymbolic("Continuous");
                propInfo.manualCapable = exposureAutoNode.hasSymbolic("Off");
                propInfo.onePushCapable = exposureAutoNode.hasSymbolic("Once");
            }
            propInfo.absoluteCapable = true;
            propInfo.onOffCapable = false;
            propInfo.readOutCapable = false;
            if (exposureTimeNode.isAvailable() && exposureTimeNode.isReadable())
            {
                propInfo.minValue = exposureTimeNode.minIntValue();
                propInfo.maxValue = exposureTimeNode.maxIntValue();
                double lowerLimit = 0.0;
                double upperLimit = 0.0;
                getShutterLimits(exposureTimeNode, lowerLimit, upperLimit);
                propInfo.minAbsoluteValue = float(lowerLimit);
                propInfo.maxAbsoluteValue = float(upperLimit);
                propInfo.haveUnits = !exposureTimeNode.unit().empty();
                propInfo.units =  exposureTimeNode.unit();
                propInfo.unitsAbbr = exposureTimeNode.unit();
            }
        }

        return propInfo;
    }


    PropertyInfo CameraDevice_arena::getPropertyInfoGain()
    {
        Node_arena gainAutoNode = cameraNode("GainAuto");
        Node_arena gainNode = cameraNode("Gain");

        PropertyInfo propInfo;
        propInfo.type = PROPERTY_TYPE_GAIN;
        propInfo.present = gainAutoNode.isAvailable() || gainNode.isAvailable();

        if (propInfo.present)
        {
            propInfo.autoCapable = false;
            propInfo.manualCapable = true;
            propInfo.onePushCapable = false;
            if (gainAutoNode.isAvailable() && gainAutoNode.isReadable())
            {
                propInfo.autoCapable = gainAutoNode.hasSymbolic("Continuous");
                propInfo.manualCapable = gainAutoNode.hasSymbolic("Off");
                propInfo.onePushCapable = gainAutoNode.hasSymbolic("Once");
            }
            propInfo.absoluteCapable = true;
            propInfo.onOffCapable = false;
            propInfo.readOutCapable = false;
            if (gainNode.isAvailable() && gainNode.isReadable())
            {
                propInfo.minValue = gainNode.minIntValue();
                propInfo.maxValue = gainNode.maxIntValue();
                propInfo.minAbsoluteValue = float(gainNode.minValue());
                propInfo.maxAbsoluteValue = float(gainNode.maxValue());
                propInfo.haveUnits = !gainNode.unit().empty();
                propInfo.units =  gainNode.unit();
                propInfo.unitsAbbr = gainNode.unit();
            }
        }

        return propInfo;
    }


    PropertyInfo CameraDevice_arena::getPropertyInfoTriggerDelay()
    {
        return getNumberPropertyInfo(PROPERTY_TYPE_TRIGGER_DELAY, "TriggerDelay", true);
    }


    PropertyInfo CameraDevice_arena::getPropertyInfoFrameRate()
    {
        return getNumberPropertyInfo(PROPERTY_TYPE_FRAME_RATE, "AcquisitionFrameRate", true);
    }


    PropertyInfo CameraDevice_arena::getPropertyInfoTemperature()
    {
        return getNumberPropertyInfo(PROPERTY_TYPE_TEMPERATURE, "DeviceTemperature", false);
    }


    PropertyInfo CameraDevice_arena::getPropertyInfoTriggerMode()
    {
        // Not implemented - dummy method
        PropertyInfo propInfo;
        propInfo.type = PROPERTY_TYPE_TRIGGER_MODE;
        propInfo.present = true;
        return propInfo;
    }


    // Get Property methods
    // ------------------------

    Property CameraDevice_arena::getPropertyBrightness()
    {
        return getNumberProperty(PROPERTY_TYPE_BRIGHTNESS, "BlackLevel");
    }


    Property CameraDevice_arena::getPropertyGamma()
    {
        Node_arena gammaNode = cameraNode("Gamma");
        Node_arena gammaEnableNode = cameraNode("GammaEnable");

        Property prop;
        prop.type = PROPERTY_TYPE_GAMMA;
        prop.present = gammaEnableNode.isAvailable() || gammaNode.isAvailable();

        if (prop.present)
        {
            prop.absoluteControl = true;
            prop.onePush = false;
            prop.autoActive = false;
            prop.on = true;
            if (gammaEnableNode.isAvailable() && gammaEnableNode.isReadable())
            {
                prop.on = gammaEnableNode.boolValue();
            }
            if (gammaNode.isAvailable() && gammaNode.isReadable())
            {
                prop.value = gammaNode.intValue();
                prop.absoluteValue = float(gammaNode.value());
            }
        }

        return prop;
    }


    Property CameraDevice_arena::getPropertyShutter()
    {
        Node_arena exposureAutoNode = cameraNode("ExposureAuto");
        Node_arena exposureTimeNode = cameraNode("ExposureTime");

        Property prop;
        prop.type = PROPERTY_TYPE_SHUTTER;
        prop.present = exposureAutoNode.isAvailable() || exposureTimeNode.isAvailable();

        if (prop.present)
        {
            prop.absoluteControl = true;
            prop.onePush = false;
            prop.autoActive = false;
            if (exposureAutoNode.isAvailable() && exposureAutoNode.isReadable())
            {
                prop.autoActive = (exposureAutoNode.symbolic() == std::string("Continuous"));
            }
            if (exposureTimeNode.isAvailable() && exposureTimeNode.isReadable())
            {
                double lowerLimit = 0.0;
                double upperLimit = 0.0;
                getShutterLimits(exposureTimeNode, lowerLimit, upperLimit);
                prop.value = exposureTimeNode.intValueWithLimits(lowerLimit, upperLimit);
                prop.absoluteValue = float(exposureTimeNode.value());
            }
        }

        return prop;
    }


    Property CameraDevice_arena::getPropertyGain()
    {
        Node_arena gainAutoNode = cameraNode("GainAuto");
        Node_arena gainNode = cameraNode("Gain");

        Property prop;
        prop.type = PROPERTY_TYPE_GAIN;
        prop.present = gainAutoNode.isAvailable() || gainNode.isAvailable();

        if (prop.present)
        {
            prop.absoluteControl = true;
            prop.onePush = false;
            prop.autoActive = false;
            if (gainAutoNode.isAvailable() && gainAutoNode.isReadable())
            {
                prop.autoActive = (gainAutoNode.symbolic() == std::string("Continuous"));
            }
            if (gainNode.isAvailable() && gainNode.isReadable())
            {
                prop.value = gainNode.intValue();
                prop.absoluteValue = float(gainNode.value());
            }
        }

        return prop;
    }


    Property CameraDevice_arena::getPropertyTriggerDelay()
    {
        return getNumberProperty(PROPERTY_TYPE_TRIGGER_DELAY, "TriggerDelay");
    }


    Property CameraDevice_arena::getPropertyFrameRate()
    {
        return getNumberProperty(PROPERTY_TYPE_FRAME_RATE, "AcquisitionFrameRate");
    }


    Property CameraDevice_arena::getPropertyTemperature()
    {
        return getNumberProperty(PROPERTY_TYPE_TEMPERATURE, "DeviceTemperature");
    }


    Property CameraDevice_arena::getPropertyTriggerMode()
    {
        // Not implemented - dummy method
        Property prop;
        prop.type = PROPERTY_TYPE_TRIGGER_MODE;
        prop.present = true;
        return prop;
    }


    // Set Property methods
    // ---------------------

    void CameraDevice_arena::setPropertyBrightness(Property prop)
    {
        setNumberProperty(prop, "BlackLevel");
    }


    void CameraDevice_arena::setPropertyGamma(Property prop)
    {
        Node_arena gammaNode = cameraNode("Gamma");
        Node_arena gammaEnableNode = cameraNode("GammaEnable");

        if (gammaEnableNode.isAvailable() && gammaEnableNode.isWritable())
        {
            gammaEnableNode.setBoolValue(prop.on);
        }
        if (gammaNode.isAvailable() && gammaNode.isReadable() && gammaNode.isWritable())
        {
            if (prop.absoluteControl)
            {
                gammaNode.setValue(prop.absoluteValue);
            }
            else
            {
                gammaNode.setValueFromInt(prop.value);
            }
        }
    }


    void CameraDevice_arena::setPropertyShutter(Property prop)
    {
        setAutoNumberProperty(prop, "ExposureAuto", "ExposureTime", true);
    }


    void CameraDevice_arena::setPropertyGain(Property prop)
    {
        setAutoNumberProperty(prop, "GainAuto", "Gain", false);
    }


    void CameraDevice_arena::setPropertyTriggerDelay(Property prop)
    {
        setNumberProperty(prop, "TriggerDelay");
    }


    void CameraDevice_arena::setPropertyFrameRate(Property prop)
    {
        setNumberProperty(prop, "AcquisitionFrameRate");
    }


    void CameraDevice_arena::setPropertyTemperature(Property prop)
    {
        // Do nothing
    }


    void CameraDevice_arena::setPropertyTriggerMode(Property prop)
    {
        // Do nothing
    }


    PixelFormat CameraDevice_arena::getPixelFormat()
    {
        Node_arena pixelFormatNode = cameraNode("PixelFormat");
        return convertPixelFormat_from_arena(uint64_t(pixelFormatNode.enumIntValue()));
    }


    bool CameraDevice_arena::isSupportedPixelFormat(PixelFormat pixelFormat, ImageMode imgMode)
    {
        PixelFormatList formatList = getListOfSupportedPixelFormats(imgMode);
        return std::find(std::begin(formatList), std::end(formatList), pixelFormat) != std::end(formatList);
    }


    // PropertyInfo, and Property dispatch maps
    // ----------------------------------------

    std::map<PropertyType, std::function<PropertyInfo(CameraDevice_arena*)>> CameraDevice_arena::getPropertyInfoDispatchMap_ =
    {
        {PROPERTY_TYPE_BRIGHTNESS,     std::function<PropertyInfo(CameraDevice_arena*)>(&CameraDevice_arena::getPropertyInfoBrightness)},
        {PROPERTY_TYPE_GAMMA,          std::function<PropertyInfo(CameraDevice_arena*)>(&CameraDevice_arena::getPropertyInfoGamma)},
        {PROPERTY_TYPE_SHUTTER,        std::function<PropertyInfo(CameraDevice_arena*)>(&CameraDevice_arena::getPropertyInfoShutter)},
        {PROPERTY_TYPE_GAIN,           std::function<PropertyInfo(CameraDevice_arena*)>(&CameraDevice_arena::getPropertyInfoGain)},
        {PROPERTY_TYPE_TRIGGER_DELAY,  std::function<PropertyInfo(CameraDevice_arena*)>(&CameraDevice_arena::getPropertyInfoTriggerDelay)},
        {PROPERTY_TYPE_FRAME_RATE,     std::function<PropertyInfo(CameraDevice_arena*)>(&CameraDevice_arena::getPropertyInfoFrameRate)},
        {PROPERTY_TYPE_TEMPERATURE,    std::function<PropertyInfo(CameraDevice_arena*)>(&CameraDevice_arena::getPropertyInfoTemperature)},
        {PROPERTY_TYPE_TRIGGER_MODE,   std::function<PropertyInfo(CameraDevice_arena*)>(&CameraDevice_arena::getPropertyInfoTriggerMode)},
    };


    std::map<PropertyType, std::function<Property(CameraDevice_arena*)>> CameraDevice_arena::getPropertyDispatchMap_ =
    {
        {PROPERTY_TYPE_BRIGHTNESS,     std::function<Property(CameraDevice_arena*)>(&CameraDevice_arena::getPropertyBrightness)},
        {PROPERTY_TYPE_GAMMA,          std::function<Property(CameraDevice_arena*)>(&CameraDevice_arena::getPropertyGamma)},
        {PROPERTY_TYPE_SHUTTER,        std::function<Property(CameraDevice_arena*)>(&CameraDevice_arena::getPropertyShutter)},
        {PROPERTY_TYPE_GAIN,           std::function<Property(CameraDevice_arena*)>(&CameraDevice_arena::getPropertyGain)},
        {PROPERTY_TYPE_TRIGGER_DELAY,  std::function<Property(CameraDevice_arena*)>(&CameraDevice_arena::getPropertyTriggerDelay)},
        {PROPERTY_TYPE_FRAME_RATE,     std::function<Property(CameraDevice_arena*)>(&CameraDevice_arena::getPropertyFrameRate)},
        {PROPERTY_TYPE_TEMPERATURE,    std::function<Property(CameraDevice_arena*)>(&CameraDevice_arena::getPropertyTemperature)},
        {PROPERTY_TYPE_TRIGGER_MODE,   std::function<Property(CameraDevice_arena*)>(&CameraDevice_arena::getPropertyTriggerMode)},
    };


    std::map<PropertyType, std::function<void(CameraDevice_arena*,Property)>> CameraDevice_arena::setPropertyDispatchMap_ =
    {
        {PROPERTY_TYPE_BRIGHTNESS,     std::function<void(CameraDevice_arena*, Property)>(&CameraDevice_arena::setPropertyBrightness)},
        {PROPERTY_TYPE_GAMMA,          std::function<void(CameraDevice_arena*, Property)>(&CameraDevice_arena::setPropertyGamma)},
        {PROPERTY_TYPE_SHUTTER,        std::function<void(CameraDevice_arena*, Property)>(&CameraDevice_arena::setPropertyShutter)},
        {PROPERTY_TYPE_GAIN,           std::function<void(CameraDevice_arena*, Property)>(&CameraDevice_arena::setPropertyGain)},
        {PROPERTY_TYPE_TRIGGER_DELAY,  std::function<void(CameraDevice_arena*, Property)>(&CameraDevice_arena::setPropertyTriggerDelay)},
        {PROPERTY_TYPE_FRAME_RATE,     std::function<void(CameraDevice_arena*, Property)>(&CameraDevice_arena::setPropertyFrameRate)},
        {PROPERTY_TYPE_TEMPERATURE,    std::function<void(CameraDevice_arena*, Property)>(&CameraDevice_arena::setPropertyTemperature)},
        {PROPERTY_TYPE_TRIGGER_MODE,   std::function<void(CameraDevice_arena*, Property)>(&CameraDevice_arena::setPropertyTriggerMode)},
    };

} // namespace bias

#endif // #ifdef WITH_ARENA
