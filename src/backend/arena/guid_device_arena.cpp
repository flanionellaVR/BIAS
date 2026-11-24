#ifdef WITH_ARENA

#include "guid_device_arena.hpp"
#include <sstream>
#include <iostream>

namespace bias {

    GuidDevice_arena::GuidDevice_arena()
    {
        value_ = std::string("-1");
    }

    GuidDevice_arena::GuidDevice_arena(std::string guid_arena)
    {
        value_ = guid_arena;
    }

    CameraLib GuidDevice_arena::getCameraLib()
    {
        return CAMERA_LIB_ARENA;
    }

    std::string GuidDevice_arena::toString()
    {
        return value_;
    }

    void GuidDevice_arena::printValue()
    {
        std::cout << "guid: " << toString() << std::endl;
    }

    std::string GuidDevice_arena::getValue()
    {
        return value_;
    }

    bool GuidDevice_arena::isEqual(GuidDevice &guid)
    {
        return value_.compare(guid.toString()) == 0;
    }

    bool GuidDevice_arena::lessThan(GuidDevice &guid)
    {
        return value_.compare(guid.toString()) < 0;
    }

    bool GuidDevice_arena::lessThanEqual(GuidDevice &guid)
    {
        return isEqual(guid) || lessThan(guid);
    }
}

#endif // #ifdef WITH_ARENA
