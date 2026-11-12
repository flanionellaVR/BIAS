# Copy the DLLs a deployed BIAS folder needs from a pixi (conda-forge)
# environment into that folder, so it runs on a machine without the
# environment.
#
# windeployqt only copies Qt's own DLLs and plugins. Qt and OpenCV from
# conda-forge load further DLLs from the environment (ICU, FFmpeg, libpng,
# ...), and so do the Qt plugins. This script resolves the dependencies of
# every .exe and .dll in DEPLOY_DIR (recursively) and copies the ones found in
# ENV_BIN_DIR into DEPLOY_DIR. It also copies the MSVC C++ runtime from
# ENV_BIN_DIR, so the folder carries the runtime that Qt and OpenCV were built
# against rather than depending on the version installed on the target
# machine.
#
# Run it after windeployqt, from a shell where dumpbin.exe is on PATH (a
# Visual Studio Developer PowerShell, or after vcvars64.bat):
#
#   cmake -DDEPLOY_DIR=<folder with test_gui.exe>
#         -DENV_BIN_DIR=<pixi env>/Library/bin
#         -P cmake/collect_runtime_dlls.cmake

cmake_minimum_required(VERSION 3.21)

foreach(var DEPLOY_DIR ENV_BIN_DIR)
    if(NOT DEFINED ${var})
        message(FATAL_ERROR "${var} is not set")
    endif()
    file(TO_CMAKE_PATH "${${var}}" ${var})
    get_filename_component(${var} "${${var}}" ABSOLUTE)
    if(NOT IS_DIRECTORY "${${var}}")
        message(FATAL_ERROR "${var} is not a directory: ${${var}}")
    endif()
endforeach()

find_program(DUMPBIN_EXECUTABLE dumpbin)
if(NOT DUMPBIN_EXECUTABLE)
    message(FATAL_ERROR "dumpbin not found on PATH; run from an MSVC developer environment")
endif()
set(CMAKE_GET_RUNTIME_DEPENDENCIES_PLATFORM "windows+pe")
set(CMAKE_GET_RUNTIME_DEPENDENCIES_TOOL "dumpbin")
set(CMAKE_GET_RUNTIME_DEPENDENCIES_COMMAND "${DUMPBIN_EXECUTABLE}")

file(GLOB executables "${DEPLOY_DIR}/*.exe")
file(GLOB_RECURSE modules "${DEPLOY_DIR}/*.dll")
if(NOT executables)
    message(FATAL_ERROR "no .exe found in ${DEPLOY_DIR}")
endif()

# DEPLOY_DIR comes first so that the Qt plugins in its subfolders resolve the
# Qt DLLs windeployqt already placed there. Anything under the Windows folder
# is part of the OS and is not deployed.
file(GET_RUNTIME_DEPENDENCIES
    EXECUTABLES ${executables}
    MODULES ${modules}
    DIRECTORIES "${DEPLOY_DIR}" "${ENV_BIN_DIR}"
    PRE_EXCLUDE_REGEXES "^api-ms-" "^ext-ms-"
    POST_EXCLUDE_REGEXES "^[A-Za-z]:[/\\\\][Ww][Ii][Nn][Dd][Oo][Ww][Ss][/\\\\]"
    RESOLVED_DEPENDENCIES_VAR resolved
    UNRESOLVED_DEPENDENCIES_VAR unresolved
    CONFLICTING_DEPENDENCIES_PREFIX conflicting
    )

# A conflict means one DLL name resolved to different files for different
# dependents. Deploy the copy from the environment if there is one.
foreach(name IN LISTS conflicting_FILENAMES)
    message(WARNING "${name} resolves to several files: ${conflicting_${name}}")
    list(APPEND resolved ${conflicting_${name}})
endforeach()

string(TOLOWER "${ENV_BIN_DIR}" env_bin_dir_lower)
set(deployed "")
foreach(dll IN LISTS resolved)
    file(TO_CMAKE_PATH "${dll}" dll)
    get_filename_component(dir "${dll}" DIRECTORY)
    get_filename_component(name "${dll}" NAME)
    string(TOLOWER "${dir}" dir_lower)
    string(TOLOWER "${name}" name_lower)
    if(dir_lower STREQUAL env_bin_dir_lower AND NOT name_lower IN_LIST deployed)
        file(COPY "${dll}" DESTINATION "${DEPLOY_DIR}")
        list(APPEND deployed "${name_lower}")
        message(STATUS "Deployed ${name}")
    endif()
endforeach()

# Windows finds the MSVC runtime in system32 before the environment, so the
# resolution above never selects the environment's copy. Deploy it explicitly
# (vcomp140 is the OpenMP runtime).
file(GLOB vc_runtime
    "${ENV_BIN_DIR}/concrt140.dll"
    "${ENV_BIN_DIR}/msvcp140*.dll"
    "${ENV_BIN_DIR}/vcomp140.dll"
    "${ENV_BIN_DIR}/vcruntime140*.dll"
    )
if(NOT vc_runtime)
    message(FATAL_ERROR "no MSVC runtime DLLs found in ${ENV_BIN_DIR}")
endif()
foreach(dll IN LISTS vc_runtime)
    get_filename_component(name "${dll}" NAME)
    file(COPY "${dll}" DESTINATION "${DEPLOY_DIR}")
    message(STATUS "Deployed ${name} (MSVC runtime)")
endforeach()

if(unresolved)
    list(REMOVE_DUPLICATES unresolved)
    message(WARNING "Unresolved dependencies (not deployed): ${unresolved}")
endif()

list(LENGTH deployed count)
message(STATUS "Copied ${count} DLLs from ${ENV_BIN_DIR} to ${DEPLOY_DIR}")
