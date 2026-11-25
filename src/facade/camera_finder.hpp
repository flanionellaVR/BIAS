#ifndef BIAS_CAMERA_FINDER_HPP
#define BIAS_CAMERA_FINDER_HPP

#include "basic_types.hpp"
#include "guid_fwd.hpp"
#include "camera_fwd.hpp"
#include <string>

#ifdef WITH_FC2
#include "FlyCapture2_C.h"
#endif
#ifdef WITH_DC1394
#include <dc1394/dc1394.h> 
#endif

namespace bias {

    class CameraFinder
    {
        public:
            CameraFinder();
            ~CameraFinder();

            unsigned int numberOfCameras();
            Guid getGuidByIndex(unsigned int index);

            GuidSet getGuidSet();
            GuidList getGuidList();

            CameraPtrSet createCameraPtrSet();
            CameraPtrList createCameraPtrList();

            std::string getGuidListAsString();
            void printGuid();

            std::string getSpinnakerVersionString();
            std::string getArenaVersionString();

            // Name and version of the camera SDK this build uses, e.g. 
            // "Spinnaker version 2.6.0.160" - for display in the GUI.
            std::string getCameraSdkVersionString();

            // True if the camera SDK is part of this build and its runtime library
            // was found on this machine and could be started.
            bool isSpinnakerAvailable();
            bool isArenaAvailable();

        private:
            GuidSet guidSet_;
            bool spinAvailable_ = false;
            bool arenaAvailable_ = false;
            
            void createQueryContext_fc2();
            void destroyQueryContext_fc2();

            void createQueryContext_dc1394();
            void destroyQueryContext_dc1394();

            void createQueryContext_spin();
            void destroyQueryContext_spin();

            void createQueryContext_arena();
            void destroyQueryContext_arena();

            void update();
            void update_fc2();
            void update_dc1394();
            void update_spin();
            void update_arena();

#ifdef WITH_FC2
        private:
            fc2Context queryContext_fc2_ = nullptr;
#endif
#ifdef WITH_DC1394
        private:
            dc1394_t *queryContext_dc1394_ = nullptr;
#endif
#ifdef WITH_SPIN
        private:
            // spinSystem handle. Kept as void* so that the Spinnaker headers are only
            // needed in camera_finder.cpp (they clash with those of the Arena SDK).
            void *queryContext_spin_ = nullptr;

#endif
#ifdef WITH_ARENA
        private:
            // True while this finder holds a reference to the shared Arena system
            bool haveQueryContext_arena_ = false;
#endif
    };

} // namespace bias

#endif // #ifndef BIAS_CAMERA_FINDER_HPP
