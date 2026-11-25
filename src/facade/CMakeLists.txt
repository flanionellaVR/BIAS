cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(bias_camera_facade)

set(
    bias_camera_facade_SOURCE 
    utils.cpp
    guid.cpp 
    camera.cpp 
    camera_finder.cpp 
    exception.cpp
    property.cpp
    format7.cpp
    runtime_library.cpp
    )

add_library(bias_camera_facade ${bias_camera_facade_SOURCE})

set(bias_camera_facade_link_libs ${bias_ext_link_libs} bias_backend_base)

if(with_fc2)
    set(bias_camera_facade_link_libs ${bias_camera_facade_link_libs} bias_backend_fc2)
endif()

if(with_dc1394)
    set(bias_camera_facade_link_libs ${bias_camera_facade_link_libs} bias_backend_dc1394)
endif()

if (with_spin)
    set(bias_camera_facade_link_libs ${bias_camera_facade_link_libs} bias_backend_spin)
endif()

if (with_arena)
    set(bias_camera_facade_link_libs ${bias_camera_facade_link_libs} bias_backend_arena)
endif()

if (with_video_backend)
    set(bias_camera_facade_link_libs ${bias_camera_facade_link_libs} bias_backend_video)
endif()

target_link_libraries(bias_camera_facade ${bias_camera_facade_link_libs})

