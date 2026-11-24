#ifdef WITH_ARENA
#include "utils_arena.hpp"
#include "exception.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <opencv2/core/core.hpp>
#include "ArenaCAPI.h"

namespace bias
{
    static const size_t MAX_BUF_LEN = 1024;

    // Time allowed for device discovery when the device list is updated (ms)
    static const uint64_t UPDATE_DEVICES_TIMEOUT_MS = 1000;


    std::string getLastErrorMessage_arena()
    {
        char buffer[MAX_BUF_LEN];
        size_t bufferLen = MAX_BUF_LEN;
        buffer[0] = '\0';
        AC_ERROR err = acGetLastErrorMessage(buffer, &bufferLen);
        if (err != AC_ERR_SUCCESS)
        {
            return std::string("(no error message available)");
        }
        buffer[MAX_BUF_LEN-1] = '\0';
        return std::string(buffer);
    }


    static std::string errorString(const char *function, const std::string &msg, AC_ERROR err)
    {
        std::stringstream ssError;
        ssError << function << ": " << msg << ", error = " << err;
        ssError << " (" << getLastErrorMessage_arena() << ")";
        return ssError.str();
    }


    // Arena system - process wide, reference counted
    // ------------------------------------------------------------------------

    static std::recursive_mutex systemMutex;
    static acSystem hSystemShared = nullptr;
    static unsigned int systemUseCount = 0;


    void* acquireSystem_arena()
    {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        if (hSystemShared == nullptr)
        {
            AC_ERROR err = acOpenSystem(&hSystemShared);
            if ((err != AC_ERR_SUCCESS) || (hSystemShared == nullptr))
            {
                hSystemShared = nullptr;
                throw RuntimeError(ERROR_ARENA_OPEN_SYSTEM, errorString(__FUNCTION__, "unable to open Arena system", err));
            }
            systemUseCount = 0;
        }
        systemUseCount++;
        return hSystemShared;
    }


    void releaseSystem_arena()
    {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        if ((hSystemShared == nullptr) || (systemUseCount == 0))
        {
            return;
        }
        systemUseCount--;
        if (systemUseCount == 0)
        {
            AC_ERROR err = acCloseSystem(hSystemShared);
            hSystemShared = nullptr;
            if (err != AC_ERR_SUCCESS)
            {
                throw RuntimeError(ERROR_ARENA_CLOSE_SYSTEM, errorString(__FUNCTION__, "unable to close Arena system", err));
            }
        }
    }


    // Updates the system's device list and returns the serial number of each device
    static std::vector<std::string> updateDeviceSerials(acSystem hSystem)
    {
        AC_ERROR err = acSystemUpdateDevices(hSystem, UPDATE_DEVICES_TIMEOUT_MS);
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_ENUMERATE_CAMERAS, errorString(__FUNCTION__, "unable to update Arena device list", err));
        }

        size_t numDevices = 0;
        err = acSystemGetNumDevices(hSystem, &numDevices);
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_ENUMERATE_CAMERAS, errorString(__FUNCTION__, "unable to get number of Arena devices", err));
        }

        std::vector<std::string> serialVec;
        for (size_t i=0; i<numDevices; i++)
        {
            char buffer[MAX_BUF_LEN];
            size_t bufferLen = MAX_BUF_LEN;
            err = acSystemGetDeviceSerial(hSystem, i, buffer, &bufferLen);
            if (err != AC_ERR_SUCCESS)
            {
                throw RuntimeError(ERROR_ARENA_ENUMERATE_CAMERAS, errorString(__FUNCTION__, "unable to get serial number of Arena device", err));
            }
            serialVec.push_back(std::string(buffer));
        }
        return serialVec;
    }


    std::vector<std::string> findCameraSerials_arena()
    {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        acSystem hSystem = acquireSystem_arena();
        std::vector<std::string> serialVec;
        try
        {
            serialVec = updateDeviceSerials(hSystem);
        }
        catch (...)
        {
            releaseSystem_arena();
            throw;
        }
        releaseSystem_arena();
        return serialVec;
    }


    void* createDevice_arena(void *hSystem, const std::string &serial)
    {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);

        std::vector<std::string> serialVec = updateDeviceSerials(hSystem);
        auto it = std::find(serialVec.begin(), serialVec.end(), serial);
        if (it == serialVec.end())
        {
            std::stringstream ssError;
            ssError << __FUNCTION__ << ": Arena camera with serial number " << serial << " not found";
            throw RuntimeError(ERROR_ARENA_CAMERA_NOT_FOUND, ssError.str());
        }
        size_t index = size_t(std::distance(serialVec.begin(), it));

        acDevice hDevice = nullptr;
        AC_ERROR err = acSystemCreateDevice(hSystem, index, &hDevice);
        if ((err != AC_ERR_SUCCESS) || (hDevice == nullptr))
        {
            throw RuntimeError(ERROR_ARENA_CREATE_DEVICE, errorString(__FUNCTION__, "unable to create Arena device (is it open in another program?)", err));
        }
        return hDevice;
    }


    void destroyDevice_arena(void *hSystem, void *hDevice)
    {
        std::lock_guard<std::recursive_mutex> lock(systemMutex);
        AC_ERROR err = acSystemDestroyDevice(hSystem, hDevice);
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_DESTROY_DEVICE, errorString(__FUNCTION__, "unable to destroy Arena device", err));
        }
    }


    std::string getVersionString_arena()
    {
        char buffer[MAX_BUF_LEN];
        size_t bufferLen = MAX_BUF_LEN;
        AC_ERROR err = acGetVersion(buffer, &bufferLen);
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_GET_LIBRARY_VERSION, errorString(__FUNCTION__, "unable to get Arena library version", err));
        }
        return std::string(buffer);
    }


    // Node_arena
    // ------------------------------------------------------------------------

    Node_arena::Node_arena() {}


    Node_arena::Node_arena(void *hNodeMap, const std::string &name)
    {
        name_ = name;
        hNode_ = nullptr;
        if (hNodeMap != nullptr)
        {
            acNode hNode = nullptr;
            AC_ERROR err = acNodeMapGetNode(hNodeMap, name.c_str(), &hNode);
            if (err == AC_ERR_SUCCESS)
            {
                hNode_ = hNode;
            }
        }
    }


    std::string Node_arena::name()
    {
        return name_;
    }


    int Node_arena::accessMode()
    {
        if (hNode_ == nullptr)
        {
            return AC_ACCESS_MODE_NI;
        }
        AC_ACCESS_MODE mode = AC_ACCESS_MODE_NI;
        AC_ERROR err = acNodeGetAccessMode(hNode_, &mode);
        if (err != AC_ERR_SUCCESS)
        {
            return AC_ACCESS_MODE_NI;
        }
        return int(mode);
    }


    int Node_arena::interfaceType()
    {
        if (hNode_ == nullptr)
        {
            return -1;
        }
        AC_INTERFACE_TYPE type = -1;
        AC_ERROR err = acNodeGetPrincipalInterfaceType(hNode_, &type);
        if (err != AC_ERR_SUCCESS)
        {
            return -1;
        }
        return int(type);
    }


    bool Node_arena::isAvailable()
    {
        int mode = accessMode();
        return (mode == AC_ACCESS_MODE_RO) || (mode == AC_ACCESS_MODE_RW) || (mode == AC_ACCESS_MODE_WO);
    }


    bool Node_arena::isReadable()
    {
        int mode = accessMode();
        return (mode == AC_ACCESS_MODE_RO) || (mode == AC_ACCESS_MODE_RW);
    }


    bool Node_arena::isWritable()
    {
        int mode = accessMode();
        return (mode == AC_ACCESS_MODE_WO) || (mode == AC_ACCESS_MODE_RW);
    }


    bool Node_arena::isInteger()
    {
        return interfaceType() == AC_INTERFACE_TYPE_INTEGER;
    }


    bool Node_arena::isFloat()
    {
        return interfaceType() == AC_INTERFACE_TYPE_FLOAT;
    }


    void Node_arena::checkReadable(const char *caller)
    {
        if (hNode_ == nullptr)
        {
            std::stringstream ssError;
            ssError << caller << ": node " << name_ << " does not exist";
            throw RuntimeError(ERROR_ARENA_NULL_HANDLE, ssError.str());
        }
        if (!isReadable())
        {
            std::stringstream ssError;
            ssError << caller << ": node " << name_ << " is not readable";
            throw RuntimeError(ERROR_ARENA_NODE_NOT_READABLE, ssError.str());
        }
    }


    void Node_arena::checkWritable(const char *caller)
    {
        if (hNode_ == nullptr)
        {
            std::stringstream ssError;
            ssError << caller << ": node " << name_ << " does not exist";
            throw RuntimeError(ERROR_ARENA_NULL_HANDLE, ssError.str());
        }
        if (!isWritable())
        {
            std::stringstream ssError;
            ssError << caller << ": node " << name_ << " is not writable";
            throw RuntimeError(ERROR_ARENA_NODE_NOT_WRITABLE, ssError.str());
        }
    }


    double Node_arena::value()
    {
        checkReadable(__FUNCTION__);
        AC_ERROR err = AC_ERR_SUCCESS;
        double val = 0.0;
        if (isInteger())
        {
            int64_t intVal = 0;
            err = acIntegerGetValue(hNode_, &intVal);
            val = double(intVal);
        }
        else if (isFloat())
        {
            err = acFloatGetValue(hNode_, &val);
        }
        else
        {
            throw RuntimeError(ERROR_ARENA_INCORRECT_NODE_TYPE, std::string(__FUNCTION__) + ": node " + name_ + " is not a number node");
        }
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_GET_NODE_VALUE, errorString(__FUNCTION__, "unable to get value of node " + name_, err));
        }
        return val;
    }


    double Node_arena::minValue()
    {
        checkReadable(__FUNCTION__);
        AC_ERROR err = AC_ERR_SUCCESS;
        double val = 0.0;
        if (isInteger())
        {
            int64_t intVal = 0;
            err = acIntegerGetMin(hNode_, &intVal);
            val = double(intVal);
        }
        else if (isFloat())
        {
            err = acFloatGetMin(hNode_, &val);
        }
        else
        {
            throw RuntimeError(ERROR_ARENA_INCORRECT_NODE_TYPE, std::string(__FUNCTION__) + ": node " + name_ + " is not a number node");
        }
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_GET_NODE_VALUE, errorString(__FUNCTION__, "unable to get minimum of node " + name_, err));
        }
        return val;
    }


    double Node_arena::maxValue()
    {
        checkReadable(__FUNCTION__);
        AC_ERROR err = AC_ERR_SUCCESS;
        double val = 0.0;
        if (isInteger())
        {
            int64_t intVal = 0;
            err = acIntegerGetMax(hNode_, &intVal);
            val = double(intVal);
        }
        else if (isFloat())
        {
            err = acFloatGetMax(hNode_, &val);
        }
        else
        {
            throw RuntimeError(ERROR_ARENA_INCORRECT_NODE_TYPE, std::string(__FUNCTION__) + ": node " + name_ + " is not a number node");
        }
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_GET_NODE_VALUE, errorString(__FUNCTION__, "unable to get maximum of node " + name_, err));
        }
        return val;
    }


    double Node_arena::increment()
    {
        checkReadable(__FUNCTION__);
        double inc = 0.0;
        if (isInteger())
        {
            int64_t intInc = 0;
            AC_ERROR err = acIntegerGetInc(hNode_, &intInc);
            if (err == AC_ERR_SUCCESS)
            {
                inc = double(intInc);
            }
        }
        else if (isFloat())
        {
            bool8_t hasInc = false;
            AC_ERROR err = acFloatHasInc(hNode_, &hasInc);
            if ((err == AC_ERR_SUCCESS) && hasInc)
            {
                double floatInc = 0.0;
                err = acFloatGetInc(hNode_, &floatInc);
                if (err == AC_ERR_SUCCESS)
                {
                    inc = floatInc;
                }
            }
        }
        return inc;
    }


    std::string Node_arena::unit()
    {
        if ((hNode_ == nullptr) || !isReadable())
        {
            return std::string("");
        }
        char buffer[MAX_BUF_LEN];
        size_t bufferLen = MAX_BUF_LEN;
        buffer[0] = '\0';
        AC_ERROR err = AC_ERR_ERROR;
        if (isInteger())
        {
            err = acIntegerGetUnit(hNode_, buffer, &bufferLen);
        }
        else if (isFloat())
        {
            err = acFloatGetUnit(hNode_, buffer, &bufferLen);
        }
        if (err != AC_ERR_SUCCESS)
        {
            return std::string("");
        }
        return std::string(buffer);
    }


    void Node_arena::setValue(double value)
    {
        checkWritable(__FUNCTION__);

        // Clamp to the node's current limits (when they can be read)
        if (isReadable())
        {
            value = std::max(std::min(value, maxValue()), minValue());
        }

        AC_ERROR err = AC_ERR_SUCCESS;
        if (isInteger())
        {
            int64_t intVal = int64_t(std::llround(value));
            if (isReadable())
            {
                // Snap down onto the node's increment grid
                int64_t intMin = int64_t(std::llround(minValue()));
                int64_t intInc = int64_t(std::llround(increment()));
                if (intInc > 1)
                {
                    intVal = intMin + ((intVal - intMin)/intInc)*intInc;
                }
            }
            err = acIntegerSetValue(hNode_, intVal);
        }
        else if (isFloat())
        {
            err = acFloatSetValue(hNode_, value);
        }
        else
        {
            throw RuntimeError(ERROR_ARENA_INCORRECT_NODE_TYPE, std::string(__FUNCTION__) + ": node " + name_ + " is not a number node");
        }
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_SET_NODE_VALUE, errorString(__FUNCTION__, "unable to set value of node " + name_, err));
        }
    }


    double Node_arena::clampToLimits(double value, double lowerLimit, double upperLimit)
    {
        value = std::max(std::min(value, upperLimit), lowerLimit);
        return std::max(std::min(value, maxValue()), minValue());
    }


    unsigned int Node_arena::intValue()
    {
        return intValueWithLimits(minValue(), maxValue());
    }


    unsigned int Node_arena::minIntValue()
    {
        return 0;
    }


    unsigned int Node_arena::maxIntValue()
    {
        return NumberOfIntValues-1;
    }


    unsigned int Node_arena::intValueWithLimits(double lowerLimit, double upperLimit)
    {
        if (!(upperLimit > lowerLimit))
        {
            return 0;
        }
        double valueClamp = clampToLimits(value(), lowerLimit, upperLimit);
        valueClamp = std::max(std::min(valueClamp, upperLimit), lowerLimit);
        return (unsigned int)(std::llround((valueClamp - lowerLimit)*double(NumberOfIntValues-1)/(upperLimit - lowerLimit)));
    }


    void Node_arena::setValueFromInt(unsigned int intValue)
    {
        setValueFromIntWithLimits(intValue, minValue(), maxValue());
    }


    void Node_arena::setValueFromIntWithLimits(unsigned int intValue, double lowerLimit, double upperLimit)
    {
        double floatValue = (upperLimit - lowerLimit)*double(intValue)/double(NumberOfIntValues-1) + lowerLimit;
        setValue(clampToLimits(floatValue, lowerLimit, upperLimit));
    }


    bool Node_arena::boolValue()
    {
        checkReadable(__FUNCTION__);
        bool8_t val = false;
        AC_ERROR err = acBooleanGetValue(hNode_, &val);
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_GET_NODE_VALUE, errorString(__FUNCTION__, "unable to get value of boolean node " + name_, err));
        }
        return val ? true : false;
    }


    void Node_arena::setBoolValue(bool value)
    {
        checkWritable(__FUNCTION__);
        AC_ERROR err = acBooleanSetValue(hNode_, value);
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_SET_NODE_VALUE, errorString(__FUNCTION__, "unable to set value of boolean node " + name_, err));
        }
    }


    std::string Node_arena::symbolic()
    {
        checkReadable(__FUNCTION__);
        char buffer[MAX_BUF_LEN];
        size_t bufferLen = MAX_BUF_LEN;
        AC_ERROR err = acEnumerationGetCurrentSymbolic(hNode_, buffer, &bufferLen);
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_GET_ENUM_ENTRY, errorString(__FUNCTION__, "unable to get current entry of enumeration node " + name_, err));
        }
        return std::string(buffer);
    }


    int64_t Node_arena::enumIntValue()
    {
        checkReadable(__FUNCTION__);
        acNode hEntry = nullptr;
        AC_ERROR err = acEnumerationGetCurrentEntry(hNode_, &hEntry);
        if ((err != AC_ERR_SUCCESS) || (hEntry == nullptr))
        {
            throw RuntimeError(ERROR_ARENA_GET_ENUM_ENTRY, errorString(__FUNCTION__, "unable to get current entry of enumeration node " + name_, err));
        }
        int64_t val = 0;
        err = acEnumEntryGetIntValue(hEntry, &val);
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_GET_ENUM_ENTRY, errorString(__FUNCTION__, "unable to get value of current entry of enumeration node " + name_, err));
        }
        return val;
    }


    std::vector<std::pair<std::string,int64_t>> Node_arena::entries()
    {
        std::vector<std::pair<std::string,int64_t>> entryVec;
        if ((hNode_ == nullptr) || !isReadable())
        {
            return entryVec;
        }

        size_t numEntries = 0;
        AC_ERROR err = acEnumerationGetNumEntries(hNode_, &numEntries);
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_GET_ENUM_ENTRY, errorString(__FUNCTION__, "unable to get number of entries of enumeration node " + name_, err));
        }

        for (size_t i=0; i<numEntries; i++)
        {
            acNode hEntry = nullptr;
            AC_ACCESS_MODE mode = AC_ACCESS_MODE_NI;
            err = acEnumerationGetEntryAndAccessModeByIndex(hNode_, i, &hEntry, &mode);
            if ((err != AC_ERR_SUCCESS) || (hEntry == nullptr))
            {
                continue;
            }
            // Skip entries which are not implemented or not available on this camera
            if ((mode != AC_ACCESS_MODE_RO) && (mode != AC_ACCESS_MODE_RW) && (mode != AC_ACCESS_MODE_WO))
            {
                continue;
            }
            char buffer[MAX_BUF_LEN];
            size_t bufferLen = MAX_BUF_LEN;
            err = acEnumEntryGetSymbolic(hEntry, buffer, &bufferLen);
            if (err != AC_ERR_SUCCESS)
            {
                continue;
            }
            int64_t val = 0;
            err = acEnumEntryGetIntValue(hEntry, &val);
            if (err != AC_ERR_SUCCESS)
            {
                continue;
            }
            entryVec.push_back(std::make_pair(std::string(buffer), val));
        }
        return entryVec;
    }


    bool Node_arena::hasSymbolic(const std::string &symbolic)
    {
        for (auto &entry : entries())
        {
            if (entry.first == symbolic)
            {
                return true;
            }
        }
        return false;
    }


    void Node_arena::setSymbolic(const std::string &symbolic)
    {
        checkWritable(__FUNCTION__);
        AC_ERROR err = acEnumerationSetBySymbolic(hNode_, symbolic.c_str());
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_SET_NODE_VALUE, errorString(__FUNCTION__, "unable to set enumeration node " + name_ + " to " + symbolic, err));
        }
    }


    void Node_arena::setEnumIntValue(int64_t value)
    {
        checkWritable(__FUNCTION__);
        AC_ERROR err = acEnumerationSetByIntValue(hNode_, value);
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_SET_NODE_VALUE, errorString(__FUNCTION__, "unable to set value of enumeration node " + name_, err));
        }
    }


    std::string Node_arena::stringValue()
    {
        checkReadable(__FUNCTION__);
        char buffer[MAX_BUF_LEN];
        size_t bufferLen = MAX_BUF_LEN;
        AC_ERROR err = acValueToString(hNode_, buffer, &bufferLen);
        if (err != AC_ERR_SUCCESS)
        {
            throw RuntimeError(ERROR_ARENA_GET_NODE_VALUE, errorString(__FUNCTION__, "unable to get value of node " + name_, err));
        }
        return std::string(buffer);
    }


    // Pixel formats
    // ------------------------------------------------------------------------

    static std::map<PixelFormat, uint64_t> pixelFormatMap_to_arena =
    {
        {PIXEL_FORMAT_MONO8,     PFNC_Mono8},
        {PIXEL_FORMAT_411YUV8,   PFNC_YUV411_8_UYYVYY},
        {PIXEL_FORMAT_422YUV8,   PFNC_YUV422_8},
        {PIXEL_FORMAT_444YUV8,   PFNC_YUV8_UYV},
        {PIXEL_FORMAT_RGB8,      PFNC_RGB8},
        {PIXEL_FORMAT_MONO16,    PFNC_Mono16},
        {PIXEL_FORMAT_RGB16,     PFNC_RGB16},
        {PIXEL_FORMAT_MONO12,    PFNC_Mono12},
        {PIXEL_FORMAT_BGR8,      PFNC_BGR8},
        {PIXEL_FORMAT_BGR16,     PFNC_BGR16},
    };


    std::vector<uint64_t> getAllowedPixelFormats_arena()
    {
        std::vector<uint64_t> formatVec;
        for (auto &kv : pixelFormatMap_to_arena)
        {
            formatVec.push_back(kv.second);
        }
        return formatVec;
    }


    bool isAllowedPixelFormat_arena(uint64_t pixFormat_arena)
    {
        for (auto &kv : pixelFormatMap_to_arena)
        {
            if (kv.second == pixFormat_arena)
            {
                return true;
            }
        }
        return false;
    }


    bool isColorPixelFormatName_arena(const std::string &symbolic)
    {
        static const std::vector<std::string> colorPrefixVec = {"Bayer", "RGB", "BGR", "YUV", "YCbCr"};
        for (auto &prefix : colorPrefixVec)
        {
            if (symbolic.compare(0, prefix.size(), prefix) == 0)
            {
                return true;
            }
        }
        return false;
    }


    uint64_t getSuitablePixelFormat_arena(uint64_t pixFormat_arena, const std::string &symbolic)
    {
        // Formats OpenCV can use directly
        if ((pixFormat_arena == PFNC_Mono8) || (pixFormat_arena == PFNC_Mono16) ||
            (pixFormat_arena == PFNC_BGR8)  || (pixFormat_arena == PFNC_BGR16))
        {
            return pixFormat_arena;
        }
        if (pixFormat_arena == PFNC_RGB16)
        {
            return PFNC_BGR16;
        }
        if (isColorPixelFormatName_arena(symbolic))
        {
            return PFNC_BGR8;
        }
        // Monochrome: the bits per pixel are in bits 16-23 of the PFNC code
        unsigned int bitsPerPixel = (unsigned int)((pixFormat_arena >> 16) & 0xff);
        return (bitsPerPixel > 8) ? uint64_t(PFNC_Mono16) : uint64_t(PFNC_Mono8);
    }


    int getCompatibleOpencvFormat_arena(uint64_t pixFormat_arena)
    {
        int opencvFormat = CV_8UC1;
        switch (pixFormat_arena)
        {
            case PFNC_Mono8:
                opencvFormat = CV_8UC1;
                break;

            case PFNC_Mono16:
                opencvFormat = CV_16UC1;
                break;

            case PFNC_BGR8:
                opencvFormat = CV_8UC3;
                break;

            case PFNC_BGR16:
                opencvFormat = CV_16UC3;
                break;

            default:
                std::stringstream ssError;
                ssError << __FUNCTION__;
                ssError << ": no compatible opencv format for pixel format 0x" << std::hex << pixFormat_arena;
                throw RuntimeError(ERROR_ARENA_NO_COMPATIBLE_OPENCV_FORMAT, ssError.str());
                break;
        }
        return opencvFormat;
    }


    uint64_t convertPixelFormat_to_arena(PixelFormat pixFormat)
    {
        if (pixelFormatMap_to_arena.count(pixFormat) == 0)
        {
            std::stringstream ssError;
            ssError << __FUNCTION__;
            ssError << ": unable to convert pixel format to Arena pixel format";
            throw RuntimeError(ERROR_ARENA_CONVERT_PIXEL_FORMAT, ssError.str());
        }
        return pixelFormatMap_to_arena[pixFormat];
    }


    PixelFormat convertPixelFormat_from_arena(uint64_t pixFormat_arena)
    {
        for (auto &kv : pixelFormatMap_to_arena)
        {
            if (kv.second == pixFormat_arena)
            {
                return kv.first;
            }
        }
        // Formats BIAS can't name (e.g. packed or Bayer formats). Capture still
        // works as images are converted in grabImage.
        return PIXEL_FORMAT_UNSPECIFIED;
    }


    // Properties
    // ------------------------------------------------------------------------

    std::vector<PropertyType> getArenaSupportedPropertyTypes()
    {
        std::vector<PropertyType> propertyTypeVec =
        {
            PROPERTY_TYPE_BRIGHTNESS,
            PROPERTY_TYPE_GAMMA,
            PROPERTY_TYPE_SHUTTER,
            PROPERTY_TYPE_GAIN,
            PROPERTY_TYPE_TRIGGER_MODE,
            PROPERTY_TYPE_TRIGGER_DELAY,
            PROPERTY_TYPE_FRAME_RATE,
            PROPERTY_TYPE_TEMPERATURE,
        };
        return propertyTypeVec;
    }


    bool isArenaSupportedPropertyType(PropertyType propType)
    {
        std::vector<PropertyType> propertyTypeVec = getArenaSupportedPropertyTypes();
        return std::find(propertyTypeVec.begin(), propertyTypeVec.end(), propType) != propertyTypeVec.end();
    }

} // namespace bias

#endif // #ifdef WITH_ARENA
