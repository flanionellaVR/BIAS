cmake_minimum_required(VERSION 3.10 FATAL_ERROR) 
project(bias)

if (WIN32)
    set(CMAKE_SYSTEM_VERSION 10 CACHE STRING INTERNAL FORCE)
    # KB 20240215 - updated to follow BIASJAABA
    set (CMAKE_SYSTEM_VERSION 10 CACHE TYPE INTERNAL FORCE)
endif()

if (POLICY CMP0020) 
    cmake_policy(SET CMP0020 NEW)
endif()


# Options
# -----------------------------------------------------------------------------
option(with_qt_gui  "include image acquisition GUI"   ON) 
option(with_fc2     "include the FlyCapture2 backend" OFF)
option(with_dc1394  "include the libdc1394 backend"   OFF)
option(with_spin    "include the spin backend"   ON)
option(with_arena   "include the Arena SDK (LUCID) backend" OFF)
option(with_tests   "include tests" ON)
option(with_video_backend     "include video backend" ON)
# KB 20240215 - follows options for BIASJAABA - include tests
#option(with_tests   "include tests" OFF)

message(STATUS "Option: with_fc2     = ${with_fc2}")
message(STATUS "Option: with_dc1394  = ${with_dc1394}")
message(STATUS "Option: with_spin    = ${with_spin}")
message(STATUS "Option: with_arena   = ${with_arena}")
message(STATUS "Option: with_qt_gui  = ${with_qt_gui}")
message(STATUS "Option: with_tests   = ${with_tests}") 
message(STATUS "Option: with_video_backend     = ${with_video_backend}")

# At least one backend must be enabled. The video backend alone is a valid
# configuration (no camera SDK needed, e.g. on CI runners): the facade compiles
# through its #else stubs and test_gui must then be started with
# -i <video-file> because there are no cameras to enumerate.
if(NOT (with_fc2 OR with_dc1394 OR with_spin OR with_arena OR with_video_backend))
    message(FATAL_ERROR
        "there must be at least one backend: enable with_fc2, with_dc1394, "
        "with_spin, with_arena or with_video_backend")
endif()

# The Spinnaker and Arena SDK backends can't be combined in one build: the camera
# SDK is chosen at compile time, e.g. -Dwith_spin=OFF -Dwith_arena=ON for LUCID
# cameras.
if(with_spin AND with_arena)
    message(FATAL_ERROR
        "with_spin and with_arena can't both be enabled: choose one camera SDK "
        "per build, e.g. -Dwith_spin=OFF -Dwith_arena=ON")
endif()

if(NOT (with_fc2 OR with_dc1394 OR with_spin OR with_arena))
    message(STATUS "No camera backend enabled: video-only build "
                   "(run test_gui with -i <video-file>)")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)

# Opencv library
# -----------------------------------------------------------------------------

find_package( OpenCV 4 REQUIRED )

if(${OpenCV_FOUND})
    message(STATUS "OpenCV found")
else()
    message(STATUS "OpenCV not found")
endif()


# FlyCapture2 library location - replace this this custom find_package
# -----------------------------------------------------------------------------
if(with_fc2)
    find_package( FlyCapture2 MODULE REQUIRED )
endif()

if(with_spin)
    find_package( Spinnaker MODULE REQUIRED )
endif()

if(with_arena)
    find_package( Arena MODULE REQUIRED )
endif()

# Qt library
# ------------------------------------------------------------------------------
if(with_qt_gui)
    set(QT_BINARY_DIR "$ENV{QTDIR}/bin")

    # Old qt 4.8 build
    # ------------------------------------------------------------
    #find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)
    # ------------------------------------------------------------

    find_package(Qt5Core REQUIRED) 
    find_package(Qt5Widgets REQUIRED)
    find_package(Qt5Gui REQUIRED) 
    find_package(Qt5Network REQUIRED)
    # KB 20240215 - added
    find_package(Qt5SerialPort REQUIRED)
    # Locate windeployqt so we can auto-deploy the Qt runtime DLLs (see README.md)
    if(WIN32)
        get_target_property(_qt5_qmake_executable Qt5::qmake IMPORTED_LOCATION)
        get_filename_component(_qt5_bin_dir "${_qt5_qmake_executable}" DIRECTORY)
        find_program(WINDEPLOYQT_EXECUTABLE windeployqt HINTS "${_qt5_bin_dir}")
        if(NOT WINDEPLOYQT_EXECUTABLE)
            message(WARNING "windeployqt not found next to qmake (${_qt5_bin_dir}); "
                             "Qt runtime DLLs will not be deployed automatically after build.")
        endif()
    endif()

endif()

# KB 20240507 - removing dependencies on HDF5

#set(HDF5_USE_STATIC_LIBRARIES ON)
#find_package(HDF5 COMPONENTS CXX)
#if(${HDF5_FOUND})
#    message(STATUS "HDF5 Found")
#else()
#    message(STATUS "HDF5 Not Found")
#endif()

# Compiler flags
# -----------------------------------------------------------------------------
#set(CMAKE_CXX_FLAGS "-std=gnu++0x -O2 -Wall")
#set(CMAKE_CXX_FLAGS "-std=gnu++0x -O2")
#set(CMAKE_CXX_FLAGS "-std=gnu++0x -mwindows -O3")
#set(CMAKE_CXX_FLAGS "-std=c++11 ")

# KB 20240215 - cuda flags follow BIASJAABA
# KB 20240507 - remove CUDA
#if(UNIX)
#    CUDA_SELECT_NVCC_ARCH_FLAGS(ARCH_FLAGS 6.1)
#    list(APPEND CUDA_NVCC_FLAGS ${ARCH_FLAGS})
#endif()

#if(WIN32)
#   set(CMAKE_CUDA_ARCH 3.5 5 6 7.5) 
#   set(CMAKE_CUDA_CODE 3.5 5 6 7.5)
#endif()

#message(STATUS "${CMAKE_CUDA_ARCH}")

#if(UNIX)
#    set(CUDA_NVCC_FLAGS_DEBUG    -Xcompiler -fPIC;-std=c++11;-g;-G CACHE STRING "" FORCE)
#    set(CUDA_NVCC_FLAGS_RELEASE  -Xcompiler -fPIC;-std=c++11;--use_fast_math;-g;-lineinfo CACHE STRING "" FORCE)
#    set(CUDA_NVCC_FLAGS_MINSIZEREL   -Xcompiler -fPIC;-std=c++11;--use_fast_math CACHE STRING "" FORCE)	
#endif()
	
#if(WIN32)
#    set(CUDA_NVCC_FLAGS_DEBUG      -g;-G CACHE STRING "" FORCE)
#    set(CUDA_NVCC_FLAGS_RELEASE    --use_fast_math;-g;-GA;-lineinfo CACHE STRING "" FORCE)
#    set(CUDA_NVCC_FLAGS_MINSIZEREL --use_fast_math CACHE STRING "" FORCE)
#endif()

# #CUDA_SELECT_NVCC_ARCH_FLAGS(ARCH_FLAGS 6.1)
# #list(APPEND CUDA_NVCC_FLAGS ${ARCH_FLAGS})
#set(CMAKE_CUDA_ARCH 3.5 5 6 7.5) 
#set(CMAKE_CUDA_CODE 3.5 5 6 7.5)

#set(CUDA_NVCC_FLAGS_DEBUG          -Xcompiler -fPIC;-std=c++11;-g;-G CACHE STRING "" FORCE)
#set(CUDA_NVCC_FLAGS_RELEASE        -Xcompiler -fPIC;-u-std=c++11;--use_fast_math;-g;-lineinfo CACHE STRING "" FORCE)
#set(CUDA_NVCC_FLAGS_MINSIZEREL     -Xcompiler -fPIC;--std=c++11;-use_fast_math CACHE STRING "" FORCE)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_C_STANDARD   99)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE RELEASE)

# Definitions 
# -----------------------------------------------------------------------------
add_definitions(-DDebug)

if(with_fc2)
    add_definitions(-DWITH_FC2)
endif()

if(with_spin)
    add_definitions(-DWITH_SPIN)
endif()

if(with_arena)
    add_definitions(-DWITH_ARENA)
endif()

# Camera SDK runtime libraries
# -----------------------------------------------------------------------------
# With MSVC the Spinnaker and Arena SDK libraries are linked "delay loaded": 
# Windows does not need them to start the executable and only loads them when
# BIAS first calls into them. Before that first call BIAS checks whether the 
# library can be loaded (isRuntimeLibraryAvailable in src/facade) and otherwise
# leaves that SDK alone. The executable therefore also starts on a machine
# without the camera SDK, e.g. to play back video files.
#
# BIAS_SPIN_RUNTIME_LIB and BIAS_ARENA_RUNTIME_LIB are the file names of the 
# libraries to check for, taken from the names of the import libraries.
set(bias_delay_load_libs "")

if(with_spin)
    get_filename_component(bias_spin_runtime_name "${Spinnaker_LIBRARY}" NAME_WE)
    if(WIN32)
        set(bias_spin_runtime_lib "${bias_spin_runtime_name}.dll")
    else()
        get_filename_component(bias_spin_runtime_lib "${Spinnaker_LIBRARY}" NAME)
    endif()
    add_compile_definitions(BIAS_SPIN_RUNTIME_LIB="${bias_spin_runtime_lib}")
    list(APPEND bias_delay_load_libs "${bias_spin_runtime_lib}")
    message(STATUS "Spinnaker runtime library: ${bias_spin_runtime_lib}")
endif()

if(with_arena)
    get_filename_component(bias_arena_runtime_name "${Arena_LIBRARY}" NAME_WE)
    if(WIN32)
        set(bias_arena_runtime_lib "${bias_arena_runtime_name}.dll")
    else()
        get_filename_component(bias_arena_runtime_lib "${Arena_LIBRARY}" NAME)
    endif()
    add_compile_definitions(BIAS_ARENA_RUNTIME_LIB="${bias_arena_runtime_lib}")
    list(APPEND bias_delay_load_libs "${bias_arena_runtime_lib}")
    message(STATUS "Arena SDK runtime library: ${bias_arena_runtime_lib}")
endif()

if(MSVC)
    foreach(bias_delay_load_lib ${bias_delay_load_libs})
        add_link_options("/DELAYLOAD:${bias_delay_load_lib}")
    endforeach()
    if(bias_delay_load_libs)
        # LNK4199: a program which makes no calls into a delay loaded library
        add_link_options("/IGNORE:4199")
    endif()
endif()

if(with_dc1394)
    add_definitions(-DWITH_DC1394)
endif()


# Include directories
# -----------------------------------------------------------------------------

include_directories("./src/backend/base")
include_directories("./src/facade")
include_directories("./src/utility")
include_directories("./src/gui")
include_directories("./src/plugin/base")
include_directories("./src/plugin/stampede")
include_directories("./src/plugin/grab_detector")
include_directories("./src/plugin/flytrack")
include_directories("./src/3rd_party/qcustomplot")

# KB 20240215 - don't compile heffalump
#if(UNIX)
#    set(HEFFALUMP_SRC_DIR /groups/branson/home/patilr/rutuja_heffalump/heffalump)
#    include_directories(${HEFFALUMP_SRC_DIR}/src/gpu)
#                        #${HEFFALUMP_SRC_DIR}/src)
#endif()
#
#if(WIN32)
#    set(HEFFALUMP_SRC_DIR C:/Users/patilr/rutuja_heffalump/heffalump)
#    include_directories(${HEFFALUMP_SRC_DIR}/src/gpu)
#                        #${HEFFALUMP_SRC_DIR}/src)
#endif()


if(with_fc2)
    include_directories(${FlyCapture2_INCLUDE_DIRS})
    include_directories("./src/backend/fc2")
endif()

if (with_spin)
    include_directories(${Spinnaker_INCLUDE_DIRS})
    include_directories("./src/backend/spin")
endif()

if (with_arena)
    # Note: the Arena SDK headers themselves are only added to the include path
    # of the Arena backend (see src/backend/arena/CMakeLists.txt).
    include_directories("./src/backend/arena")
endif()

if(with_dc1394)
    include_directories("./src/backend/dc1394")
    # Add custom libdc1394 
    #include_directories("/home/wbd/local/include")
    #link_directories("/home/wbd/local/lib")
endif()

if(with_video_backend)
    include_directories("./src/backend/video")
endif()

# External link libraries
# -----------------------------------------------------------------------------
set(bias_ext_link_LIBS ${OpenCV_LIBS})
message(STATUS "${OPENCV_LIBS}")


if(with_fc2)
    set(bias_ext_link_LIBS ${bias_ext_link_LIBS} ${FlyCapture2_LIBRARIES})
endif()

if (with_spin)
    set(bias_ext_link_LIBS ${bias_ext_link_LIBS} ${Spinnaker_LIBRARIES})
endif()

if (with_arena)
    set(bias_ext_link_LIBS ${bias_ext_link_LIBS} ${Arena_LIBRARIES})
endif()

if(with_dc1394)
    if (WIN32)
        set(bias_ext_link_LIBS ${bias_ext_link_LIBS} dc1394 1394camera setupapi)
    else()
        set(bias_ext_link_LIBS ${bias_ext_link_LIBS} dc1394)
    endif()
endif()
set(bias_ext_link_LIBS ${bias_ext_link_LIBS})

if(MSVC AND bias_delay_load_libs)
    # Helper code for delay loaded libraries (see "Camera SDK runtime libraries")
    set(bias_ext_link_LIBS ${bias_ext_link_LIBS} delayimp)
endif()

message(STATUS "${bias_ext_link_LIBS}")


# Output directory for executables - puts executables in build directory
# -----------------------------------------------------------------------------
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Add project subdirectories
# -----------------------------------------------------------------------------
add_subdirectory("src/backend/base") 

if(with_fc2) 
    add_subdirectory("src/backend/fc2") 
endif()

if(with_dc1394) 
    add_subdirectory("src/backend/dc1394")
endif() 

if(with_spin)
    add_subdirectory("src/backend/spin")
endif()

if(with_arena)
    add_subdirectory("src/backend/arena")
endif()

if(with_video_backend)
    add_subdirectory("src/backend/video")
endif()

add_subdirectory("src/facade")
add_subdirectory("src/utility")
add_subdirectory("src/plugin/base")
add_subdirectory("src/plugin/stampede")
add_subdirectory("src/plugin/grab_detector")
add_subdirectory("src/plugin/flytrack")
add_subdirectory("src/3rd_party/qcustomplot")

if(with_qt_gui)
    add_subdirectory("src/gui")
endif()

if (with_tests)
    add_subdirectory("src/test")
endif()

