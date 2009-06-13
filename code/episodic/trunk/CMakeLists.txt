cmake_minimum_required (VERSION 2.6)
project (CogX)

get_filename_component(COGX_ROOT . ABSOLUTE)

# set the output directory.
set(OUTPUT ${COGX_ROOT}/output CACHE PATH "Path prefix for installation")
set(CMAKE_INSTALL_PREFIX "${OUTPUT}" CACHE INTERNAL "Prefix prepended to install directories" FORCE)


# use cmake files packaged with CAST as well
set(CMAKE_MODULE_PATH /usr/local/share/cast/cmake;/opt/local/share/cast/cmake ${CMAKE_MODULE_PATH})

find_package(CAST REQUIRED)
include(UseCAST)
include(CASTBuild)

# use cmake files packaged with CAST as well
set(CMAKE_MODULE_PATH "${COGX_ROOT}/cmake;${CAST_CMAKE_DIR}" ${CMAKE_MODULE_PATH})

add_definitions(-Wall)


add_subdirectory (tools/d-lib)

option(BUILD_SA_NAV "Build navigation subarchitecure" YES)
if(BUILD_SA_NAV)
add_subdirectory (subarchitectures/nav.sa)
endif(BUILD_SA_NAV)


option(BUILD_HAL_ROBOT "Build hardware robotbase" YES)
if(BUILD_HAL_ROBOT)
add_subdirectory (tools/hardware/robotbase)
endif(BUILD_HAL_ROBOT)

option(BUILD_HAL_LASER "Build hardware laser" YES)
if(BUILD_HAL_LASER)
add_subdirectory (tools/hardware/laser)
endif(BUILD_HAL_LASER)

option(BUILD_HAL_TTS "Build hardware tts" NO)
if(BUILD_HAL_TTS)
add_subdirectory (tools/hardware/tts)
endif(BUILD_HAL_TTS)

option(BUILD_HAL_PTZ "Build hardware ptz" NO)
if(BUILD_HAL_PTZ)
add_subdirectory (tools/hardware/ptz)
endif(BUILD_HAL_PTZ)

option(BUILD_HAL_UTILS "Build hardware utils" YES)
if(BUILD_HAL_UTILS)
add_subdirectory (tools/hardware/utils)
endif(BUILD_HAL_UTILS)

option(BUILD_HAL_TESTS "Build hardware tests" NO)
if(BUILD_HAL_TESTS)
add_subdirectory (tools/hardware/tests)
endif(BUILD_HAL_TESTS)




add_subdirectory (tools/math)

# Spatial
option(BUILD_SA_SPATIAL "Build spatial subarchitecure" YES)
if(BUILD_SA_SPATIAL)
    add_subdirectory (subarchitectures/spatial.sa)
endif(BUILD_SA_SPATIAL)


# Celm
option(BUILD_SA_CELM "Build CAST ELM subarchitecure" YES)
if(BUILD_SA_CELM)
    add_subdirectory (subarchitectures/celm)
endif(BUILD_SA_CELM)

