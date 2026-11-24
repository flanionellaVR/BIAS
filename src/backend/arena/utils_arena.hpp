#ifdef WITH_ARENA
#ifndef BIAS_UTILS_ARENA_HPP
#define BIAS_UTILS_ARENA_HPP

// ----------------------------------------------------------------------------
// Helpers for the Arena SDK (LUCID Vision Labs) backend.
//
// Note: no Arena SDK header is included here on purpose. The SDK handles
// (acSystem, acDevice, acNodeMap, acNode, acBuffer) are all "void*" typedefs,
// so plain void* is used in the interfaces and the SDK headers stay confined
// to the .cpp files of this backend. This keeps the SDK's global typedefs
// (e.g. bool8_t, which clashes with Spinnaker's) out of the rest of BIAS.
// ----------------------------------------------------------------------------

#include "basic_types.hpp"
#include "property.hpp"
#include <cstdint>
#include <string>
#include <vector>
#include <utility>

namespace bias
{

    // Arena system - process wide, reference counted
    // ------------------------------------------------------------------------
    // The Arena SDK only allows a single open system per process, whereas BIAS
    // wants one "context" for the camera finder and one per camera device.
    // acquireSystem_arena opens the system on first use and hands out the
    // shared handle, releaseSystem_arena closes it when the last user is done.

    void* acquireSystem_arena();

    void releaseSystem_arena();

    // Serial numbers of all cameras currently visible to the Arena SDK.
    std::vector<std::string> findCameraSerials_arena();

    // Creates (opens) the device with the given serial number.
    void* createDevice_arena(void *hSystem, const std::string &serial);

    void destroyDevice_arena(void *hSystem, void *hDevice);

    std::string getVersionString_arena();

    // Text of the last error reported by the Arena SDK
    std::string getLastErrorMessage_arena();


    // Node wrapper
    // ------------------------------------------------------------------------
    // Thin wrapper around a GenICam node. One class covers all the node types
    // the backend needs (integer, float, boolean, enumeration, string); the
    // number methods work for both integer and float nodes.

    class Node_arena
    {
        public:

            static const unsigned int NumberOfIntValues = 100;

            Node_arena();
            Node_arena(void *hNodeMap, const std::string &name);

            std::string name();
            bool isAvailable();
            bool isReadable();
            bool isWritable();

            bool isInteger();
            bool isFloat();

            // Number (integer or float) nodes
            double value();
            double minValue();
            double maxValue();
            double increment();        // 0 if the node has no increment
            std::string unit();
            void setValue(double value);  // clamped to [min,max], snapped to increment

            // Number nodes mapped onto the integer range [0, NumberOfIntValues-1]
            unsigned int intValue();
            unsigned int minIntValue();
            unsigned int maxIntValue();
            unsigned int intValueWithLimits(double lowerLimit, double upperLimit);
            void setValueFromInt(unsigned int intValue);
            void setValueFromIntWithLimits(unsigned int intValue, double lowerLimit, double upperLimit);

            // Boolean nodes
            bool boolValue();
            void setBoolValue(bool value);

            // Enumeration nodes
            std::string symbolic();
            int64_t enumIntValue();
            bool hasSymbolic(const std::string &symbolic);
            void setSymbolic(const std::string &symbolic);
            void setEnumIntValue(int64_t value);
            std::vector<std::pair<std::string,int64_t>> entries();  // available entries only

            // String nodes (also returns the value of any other node as text)
            std::string stringValue();

        private:

            void *hNode_ = nullptr;
            std::string name_;

            int accessMode();
            int interfaceType();
            void checkReadable(const char *caller);
            void checkWritable(const char *caller);
            double clampToLimits(double value, double lowerLimit, double upperLimit);
    };


    // Pixel formats
    // ------------------------------------------------------------------------

    // Pixel formats (as PFNC codes) which can be selected from within BIAS
    std::vector<uint64_t> getAllowedPixelFormats_arena();

    bool isAllowedPixelFormat_arena(uint64_t pixFormat_arena);

    // True if the name of a camera pixel format denotes a color (or Bayer) format
    bool isColorPixelFormatName_arena(const std::string &symbolic);

    // PFNC code of the format images of the given camera pixel format should
    // be converted to before they are handed to OpenCV. Returns the code of
    // format itself when no conversion is required.
    uint64_t getSuitablePixelFormat_arena(uint64_t pixFormat_arena, const std::string &symbolic);

    int getCompatibleOpencvFormat_arena(uint64_t pixFormat_arena);

    uint64_t convertPixelFormat_to_arena(PixelFormat pixFormat);

    PixelFormat convertPixelFormat_from_arena(uint64_t pixFormat_arena);


    // Properties
    // ------------------------------------------------------------------------
    std::vector<PropertyType> getArenaSupportedPropertyTypes();

    bool isArenaSupportedPropertyType(PropertyType propType);

} // namespace bias

#endif // #ifndef BIAS_UTILS_ARENA_HPP
#endif // #ifdef WITH_ARENA
