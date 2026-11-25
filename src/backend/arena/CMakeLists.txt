cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(bias_backend_arena)

set(
    bias_backend_arena_SOURCE
    utils_arena.cpp
    guid_device_arena.cpp
    camera_device_arena.cpp
    )

add_library(bias_backend_arena ${bias_backend_arena_SOURCE})

# The Arena SDK headers are only needed by the sources of this backend
target_include_directories(bias_backend_arena PRIVATE ${Arena_INCLUDE_DIRS})

target_link_libraries(
    bias_backend_arena
    ${bias_ext_link_LIBS}
    bias_backend_base
    bias_camera_facade
    )
