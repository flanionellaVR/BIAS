# - Try to find the Arena SDK (LUCID Vision Labs), C API
#
# Once done this will define
#
#  Arena_FOUND         - System has the Arena SDK
#  Arena_INCLUDE_DIRS  - The Arena SDK (C API) include directories
#  Arena_LIBRARIES     - The libraries needed to use the Arena SDK C API
#  Arena_RUNTIME_DIR   - Directory with the Arena SDK runtime DLLs (Windows)
#
# Set Arena_ROOT_DIR (cmake -DArena_ROOT_DIR=...) if the SDK is not installed
# in its default location.
#
# ------------------------------------------------------------------------------

if (WIN32)
    set(typical_arena_dir "C:/Program Files/Lucid Vision Labs/Arena SDK")
    set(typical_arena_inc_dir "${typical_arena_dir}/include/ArenaC")
    set(typical_arena_lib_dir "${typical_arena_dir}/lib64/ArenaC")
    set(typical_arena_bin_dir "${typical_arena_dir}/x64Release")
else()
    # Linux: the SDK is unpacked by hand, e.g. into /opt/ArenaSDK_Linux_x64
    set(typical_arena_dir "/opt/ArenaSDK_Linux_x64")
    set(typical_arena_inc_dir "${typical_arena_dir}/include/ArenaC")
    set(typical_arena_lib_dir "${typical_arena_dir}/lib64")
    set(typical_arena_bin_dir "${typical_arena_dir}/lib64")
endif()

find_path(
    Arena_INCLUDE_DIR
    "ArenaCAPI.h"
    HINTS
    "${Arena_ROOT_DIR}/include/ArenaC"
    ${typical_arena_inc_dir}
    )

if(WIN32)
    find_library(
        Arena_LIBRARY
        NAMES "ArenaC_v140.lib" "ArenaC_v140"
        HINTS
        "${Arena_ROOT_DIR}/lib64/ArenaC"
        ${typical_arena_lib_dir}
        )
    find_path(
        Arena_RUNTIME_DIR
        "ArenaC_v140.dll"
        HINTS
        "${Arena_ROOT_DIR}/x64Release"
        ${typical_arena_bin_dir}
        )
else()
    find_library(
        Arena_LIBRARY
        NAMES "arenac" "libarenac.so"
        HINTS
        "${Arena_ROOT_DIR}/lib64"
        ${typical_arena_lib_dir}
        )
    set(Arena_RUNTIME_DIR ${typical_arena_bin_dir})
endif()

set(Arena_LIBRARIES ${Arena_LIBRARY})
set(Arena_INCLUDE_DIRS ${Arena_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set Arena_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(
    Arena  DEFAULT_MSG
    Arena_LIBRARY
    Arena_INCLUDE_DIR
    )

mark_as_advanced(Arena_INCLUDE_DIR Arena_LIBRARY Arena_RUNTIME_DIR)
