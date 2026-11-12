cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

if(with_fc2)
    project(bias_test_fc2)
    add_executable(test_fc2 test_fc2.cpp)
    target_link_libraries(test_fc2 ${bias_ext_link_LIBS} bias_camera_facade)

endif()

if(with_dc1394) 
    project(bias_test_dc1394)
    add_EXECUTABLE(test_dc1394 test_dc1394.cpp)
    target_link_libraries(test_dc1394 ${bias_ext_link_LIBS} bias_camera_facade)
endif()


if (with_spin)
    project(bias_test_spin)
    add_EXECUTABLE(test_spin test_spinnaker.cpp)
    target_link_libraries(test_spin ${bias_ext_link_LIBS} bias_camera_facade)
endif()


# Serial test
# ---------------------------------------------------------------------------------------
#project(bias_test_serial)
#if (POLICY CMP0020)
#    cmake_policy(SET CMP0020 NEW)
#endif()
#
#set(
#    bias_test_serial_SOURCES 
#    test_serial.cpp
#    )
#qt5_wrap_cpp(bias_test_serial_HEADERS_MOC ${bias_test_serial_HEADERS})
#
#add_EXECUTABLE(
#    test_serial 
#    ${bias_test_serial_HEADERS_MOC}
#    ${bias_test_serial_SOURCES} 
#    )
#qt5_use_modules(test_serial Core Gui Widgets SerialPort)


# nano_ssr_pulse
# ---------------------------------------------------------------------------------------
#project(bias_test_nano_ssr_pulse)
#if (POLICY CMP0020)
#    cmake_policy(SET CMP0020 NEW)
#endif()
#
#set(
#    bias_test_nano_ssr_pulse_HEADERS 
#    nano_ssr_pulse.hpp
#    )
#
#set(
#    bias_test_nano_ssr_pulse_SOURCES 
#    nano_ssr_pulse.cpp
#    test_nano_ssr_pulse.cpp
#    )
#qt5_wrap_cpp(bias_test_nano_ssr_pulse_HEADERS_MOC ${bias_test_nano_ssr_pulse_HEADERS})
#
#add_EXECUTABLE(
#    test_nano_ssr_pulse 
#    ${bias_test_nano_ssr_pulse_HEADERS_MOC}
#    ${bias_test_nano_ssr_pulse_SOURCES} 
#    )
#qt5_use_modules(test_nano_ssr_pulse Core Gui Widgets SerialPort)



# panels_controller
# ---------------------------------------------------------------------------------------
#project(bias_test_panels_controller)
#if (POLICY CMP0020)
#    cmake_policy(SET CMP0020 NEW)
#endif()
#
#set(
#    bias_test_panels_controller_HEADERS 
#    panels_controller.hpp
#    )
#
#set(
#    bias_test_panels_controller_SOURCES 
#    panels_controller.cpp
#    test_panels_controller.cpp
#    )
#qt5_wrap_cpp(bias_test_panels_controller_HEADERS_MOC ${bias_test_panels_controller_HEADERS})
#
#add_EXECUTABLE(
#    test_panels_controller 
#    ${bias_test_panels_controller_HEADERS_MOC}
#    ${bias_test_panels_controller_SOURCES} 
#    )
#qt5_use_modules(test_panels_controller Core Gui Widgets SerialPort)


# OpenCV DLLs next to test_gui.exe (Windows)
# ---------------------------------------------------------------------------------------
# test_gui.exe only starts if the OpenCV DLLs are next to it or on PATH, and
# windeployqt (src/gui) deploys Qt only. Copy every opencv_*.dll from the
# directories of the imported OpenCV targets. That includes
# opencv_videoio_ffmpeg*.dll, which OpenCV loads at run time to read videos
# and which no target links against.
if(WIN32 AND TARGET test_gui)
    set(bias_opencv_dll_dirs "")
    foreach(lib IN LISTS OpenCV_LIBS)
        if(TARGET ${lib})
            foreach(prop IMPORTED_LOCATION IMPORTED_LOCATION_RELEASE
                    IMPORTED_LOCATION_RELWITHDEBINFO IMPORTED_LOCATION_MINSIZEREL
                    IMPORTED_LOCATION_DEBUG)
                get_target_property(location ${lib} ${prop})
                if(location MATCHES "\\.[dD][lL][lL]$")
                    get_filename_component(location_dir "${location}" DIRECTORY)
                    list(APPEND bias_opencv_dll_dirs "${location_dir}")
                endif()
            endforeach()
        endif()
    endforeach()

    set(bias_opencv_dlls "")
    if(bias_opencv_dll_dirs)
        list(REMOVE_DUPLICATES bias_opencv_dll_dirs)
        foreach(dll_dir IN LISTS bias_opencv_dll_dirs)
            file(GLOB dlls "${dll_dir}/opencv_*.dll")
            list(APPEND bias_opencv_dlls ${dlls})
        endforeach()
    endif()

    if(bias_opencv_dlls)
        add_custom_target(copy_opencv_dlls ALL
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${bias_opencv_dlls} "$<TARGET_FILE_DIR:test_gui>"
            COMMENT "Copying OpenCV DLLs next to test_gui.exe"
            VERBATIM
            )
        # The output directory only exists once test_gui has been built.
        add_dependencies(copy_opencv_dlls test_gui)
    else()
        message(STATUS "No OpenCV DLLs found for the imported OpenCV targets: "
                       "put the OpenCV bin directory on PATH to run test_gui.exe")
    endif()
endif()


# FlyTrack smoke test
# ---------------------------------------------------------------------------------------
# Tracks the recorded video in testdata/ with test_gui --batch and compares the
# trajectory against testdata/trx_reference.json (see testdata/README.md).
# Three tests, chained with fixtures so a failure shows which step broke:
#   flytrack_smoke_clean    remove the trajectory of an earlier run
#   flytrack_smoke_run      track the sample video unattended
#   flytrack_smoke_compare  compare the trajectory against the reference
# Run them with `ctest -L smoke` (add `-C Release` for Visual Studio builds).
set(flytrack_smoke_tolerance "0.01" CACHE STRING
    "FlyTrack smoke test: largest allowed per-frame difference from the reference (pixels for x, y, a, b; radians for theta)")
set(flytrack_smoke_timeout "300" CACHE STRING
    "FlyTrack smoke test: timeout in seconds for the test_gui --batch run")

set(bias_testdata_dir "${bias_SOURCE_DIR}/testdata")
set(bias_smoke_missing "")
foreach(name sample.avi background.png flytrack_config.json trx_reference.json compare_trx.py)
    if(NOT EXISTS "${bias_testdata_dir}/${name}")
        list(APPEND bias_smoke_missing ${name})
    endif()
endforeach()

if(NOT (TARGET test_gui AND with_video_backend))
    message(STATUS "FlyTrack smoke test: not registered, needs with_qt_gui and with_video_backend")
elseif(bias_smoke_missing)
    string(REPLACE ";" ", " bias_smoke_missing "${bias_smoke_missing}")
    message(STATUS "FlyTrack smoke test: not registered, testdata/ has no "
                   "${bias_smoke_missing} (see testdata/README.md, then re-run cmake)")
else()
    find_package(Python3 COMPONENTS Interpreter)
    if(NOT Python3_Interpreter_FOUND)
        message(STATUS "FlyTrack smoke test: not registered, no Python 3 interpreter found")
    else()
        set(bias_smoke_trx "${CMAKE_CURRENT_BINARY_DIR}/flytrack_smoke_trx.json")
        if(CMAKE_VERSION VERSION_LESS 3.17)
            set(bias_rm_file remove -f)
        else()
            set(bias_rm_file rm -f)
        endif()

        add_test(NAME flytrack_smoke_clean
            COMMAND ${CMAKE_COMMAND} -E ${bias_rm_file} "${bias_smoke_trx}"
            )
        # Relative paths in flytrack_config.json (the background image) resolve
        # against the working directory, so run from testdata/.
        add_test(NAME flytrack_smoke_run
            COMMAND $<TARGET_FILE:test_gui> --batch
                    -i sample.avi -c flytrack_config.json -o "${bias_smoke_trx}"
            WORKING_DIRECTORY "${bias_testdata_dir}"
            )
        add_test(NAME flytrack_smoke_compare
            COMMAND "${Python3_EXECUTABLE}" "${bias_testdata_dir}/compare_trx.py"
                    "${bias_smoke_trx}" "${bias_testdata_dir}/trx_reference.json"
                    --tolerance ${flytrack_smoke_tolerance}
            )

        set_tests_properties(flytrack_smoke_clean PROPERTIES
            FIXTURES_SETUP flytrack_smoke_clean
            LABELS smoke
            )
        set_tests_properties(flytrack_smoke_run PROPERTIES
            ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
            TIMEOUT ${flytrack_smoke_timeout}
            FIXTURES_REQUIRED flytrack_smoke_clean
            FIXTURES_SETUP flytrack_smoke_trx
            LABELS smoke
            )
        set_tests_properties(flytrack_smoke_compare PROPERTIES
            FIXTURES_REQUIRED flytrack_smoke_trx
            LABELS smoke
            )
        message(STATUS "FlyTrack smoke test: registered (ctest -L smoke)")
    endif()
endif()

