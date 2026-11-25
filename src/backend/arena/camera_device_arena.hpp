#ifdef WITH_ARENA
#ifndef BIAS_CAMERA_DEVICE_ARENA_HPP
#define BIAS_CAMERA_DEVICE_ARENA_HPP

#include <map>
#include <string>
#include <vector>
#include <functional>
#include <cstdint>
#include <opencv2/core/core.hpp>

#include "utils.hpp"
#include "camera_device.hpp"
#include "property.hpp"
#include "utils_arena.hpp"

namespace bias {

    // Camera device for LUCID Vision Labs cameras, built on the C API of the
    // Arena SDK (ArenaC).
    //
    // Note: no Arena SDK header is included here on purpose - SDK handles are
    // kept as void* and the SDK headers are confined to the .cpp files of this
    // backend (see utils_arena.hpp).

    class CameraDevice_arena : public CameraDevice
    {
        public:

            CameraDevice_arena();
            explicit CameraDevice_arena(Guid guid);
            virtual ~CameraDevice_arena();

            virtual CameraLib getCameraLib();

            virtual void connect();
            virtual void disconnect();

            virtual void startCapture();
            virtual void stopCapture();

            virtual cv::Mat grabImage();
            virtual void grabImage(cv::Mat &image);

            virtual bool isColor();

            virtual bool isSupported(VideoMode vidMode, FrameRate frmRate);
            virtual bool isSupported(ImageMode imgMode);
            virtual unsigned int getNumberOfImageMode();

            virtual VideoMode getVideoMode();
            virtual FrameRate getFrameRate();
            virtual ImageMode getImageMode();

            virtual VideoModeList getAllowedVideoModes();
            virtual FrameRateList getAllowedFrameRates(VideoMode vidMode);
            virtual ImageModeList getAllowedImageModes();

            virtual PropertyInfo getPropertyInfo(PropertyType propType);
            virtual Property getProperty(PropertyType propType);
            virtual void setProperty(Property prop);

            virtual Format7Settings getFormat7Settings();
            virtual Format7Info getFormat7Info(ImageMode imgMode);

            virtual bool validateFormat7Settings(Format7Settings settings);
            virtual void setFormat7Configuration(Format7Settings settings, float percentSpeed);

            virtual PixelFormatList getListOfSupportedPixelFormats(ImageMode imgMode);

            virtual void setTriggerInternal();
            virtual void setTriggerExternal();
            virtual TriggerType getTriggerType();

            virtual std::string getVendorName();
            virtual std::string getModelName();

            virtual TimeStamp getImageTimeStamp();

            virtual std::string toString();

            virtual void printGuid();

            virtual void printInfo();

            // Names of all nodes in the camera's (device) node map along with
            // their current values - for development and diagnostics.
            std::string nodeMapToString();

            // Constants
            // --------------------------------------------------------
            // Artificial shutter limits to make GUI more usable.
            static constexpr double MinAllowedShutterUs = 100.0;
            static constexpr double MaxAllowedShutterUs = 200000.0;

            // Time grabImage waits for a frame before returning an empty image.
            // The image grabber holds the camera lock while grabbing, so this
            // bounds how long e.g. a property change from the GUI has to wait.
            static constexpr uint64_t GrabTimeoutMs = 100;

            // Input line used for the external trigger
            static const std::string ExternalTriggerSource;

        private:

            void *hSystem_ = nullptr;         // acSystem
            void *hDevice_ = nullptr;         // acDevice
            void *hNodeMapCamera_ = nullptr;  // acNodeMap, device (camera) features
            void *hNodeMapTLStream_ = nullptr;// acNodeMap, transport layer stream features

            std::string vendorName_;
            std::string modelName_;
            std::string serialNumber_;

            TimeStamp timeStamp_ = {0,0};
            uint64_t timeStamp_ns_ = 0;

            TriggerType triggerType_ =  TRIGGER_TYPE_UNSPECIFIED;

            // Set in startCapture - pixel format can't change while streaming
            std::string capturePixelFormatName_;

            Node_arena cameraNode(const std::string &name);
            Node_arena streamNode(const std::string &name);

            void setupStream();

            // Get Property Info methods
            static std::map<PropertyType, std::function<PropertyInfo(CameraDevice_arena*)>> getPropertyInfoDispatchMap_;

            PropertyInfo getPropertyInfoBrightness();
            PropertyInfo getPropertyInfoGamma();
            PropertyInfo getPropertyInfoShutter();
            PropertyInfo getPropertyInfoGain();
            PropertyInfo getPropertyInfoTriggerDelay();
            PropertyInfo getPropertyInfoFrameRate();
            PropertyInfo getPropertyInfoTemperature();
            PropertyInfo getPropertyInfoTriggerMode();

            // Get property methods
            static std::map<PropertyType, std::function<Property(CameraDevice_arena*)>> getPropertyDispatchMap_;

            Property getPropertyBrightness();
            Property getPropertyGamma();
            Property getPropertyShutter();
            Property getPropertyGain();
            Property getPropertyTriggerDelay();
            Property getPropertyFrameRate();
            Property getPropertyTemperature();
            Property getPropertyTriggerMode();

            // Set Property methods
            bool isPropertySettable(PropertyType propType, std::string &msg);
            static std::map<PropertyType, std::function<void(CameraDevice_arena*,Property)>> setPropertyDispatchMap_;

            void setPropertyBrightness(Property prop);
            void setPropertyGamma(Property prop);
            void setPropertyShutter(Property prop);
            void setPropertyGain(Property prop);
            void setPropertyTriggerDelay(Property prop);
            void setPropertyFrameRate(Property prop);
            void setPropertyTemperature(Property prop);
            void setPropertyTriggerMode(Property prop);

            // Helpers shared by the property methods
            void getShutterLimits(Node_arena &exposureTimeNode, double &lowerLimit, double &upperLimit);
            PropertyInfo getNumberPropertyInfo(PropertyType propType, const std::string &nodeName, bool manualCapable);
            Property getNumberProperty(PropertyType propType, const std::string &nodeName);
            void setNumberProperty(Property prop, const std::string &nodeName);
            void setAutoNumberProperty(Property prop, const std::string &autoNodeName, const std::string &nodeName, bool useShutterLimits);

            PixelFormat getPixelFormat();
            bool isSupportedPixelFormat(PixelFormat pixelFormat, ImageMode imgMode);

    };


    typedef std::shared_ptr<CameraDevice_arena> CameraDevicePtr_arena;

}

#endif // #ifndef BIAS_CAMERA_DEVICE_ARENA_HPP
#endif // #ifdef WITH_ARENA
