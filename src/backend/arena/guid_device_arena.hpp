#ifdef WITH_ARENA
#ifndef BIAS_GUID_DEVICE_ARENA_HPP
#define BIAS_GUID_DEVICE_ARENA_HPP

#include <string>
#include <memory>
#include "basic_types.hpp"
#include "guid_device.hpp"

namespace bias {

    class GuidDevice_arena : public GuidDevice
    {
        // ---------------------------------------------------------------------
        // Provides representation of Arena SDK (LUCID) specific camera guids.
        // The guid is the camera's serial number.
        //
        // Note: no Arena SDK header is included here on purpose - the SDK
        // headers are confined to the .cpp files of this backend.
        // ---------------------------------------------------------------------
        public:
            GuidDevice_arena();
            explicit GuidDevice_arena(std::string guid_arena);
            virtual ~GuidDevice_arena() {};
            virtual CameraLib getCameraLib();
            virtual void printValue();
            virtual std::string toString();
            std::string getValue();

        private:
            std::string value_;
            virtual bool isEqual(GuidDevice &guid);
            virtual bool lessThan(GuidDevice &guid);
            virtual bool lessThanEqual(GuidDevice &guid);
    };

    typedef std::shared_ptr<GuidDevice_arena> GuidDevicePtr_arena;
}

#endif // #ifndef BIAS_GUID_DEVICE_ARENA_HPP
#endif // #ifdef  WITH_ARENA
