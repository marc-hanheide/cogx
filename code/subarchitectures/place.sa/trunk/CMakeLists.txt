cmake_minimum_required (VERSION 2.6)
project (PLACESA)

# Set build type
set(CMAKE_BUILD_TYPE Release)

# You can prepare verbose Makefile to see what's going.
set(CMAKE_VERBOSE_MAKEFILE true)

# Find QT4 
find_package( Qt4 REQUIRED )
include(${QT_USE_FILE})

# Include dirs
include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/robotbase/src/c++/autogen)
include_directories(src/c++)
include_directories(src/c++/autogen)



# -----------------------------------------------------------------------
# Subdirs
# -----------------------------------------------------------------------

# Auto generated code
add_subdirectory (src/c++/autogen)

# The following tools and components are mandatory for Placa.SA
add_subdirectory(src/c++/shared)
add_subdirectory(src/c++/tools/libsvm-mod)
add_subdirectory(src/c++/components/Controller)
add_subdirectory(src/c++/components/DataProvider)
#add_subdirectory(src/c++/components/DataSaver)
add_subdirectory(src/c++/components/CueIntegrator)
#add_subdirectory(src/c++/components/NodeLabeller)

# Options
option(BUILD_SA_PLACE_LASER "Build Place.SA laser path" YES)
option(BUILD_SA_PLACE_VISION "Build Place.SA visual path" YES)
option(BUILD_SA_PLACE_AUX "Build Place.SA auxiliary components" NO)
option(BUILD_SA_PLACE_TESTING "Build Place.SA testing components" YES)

# Laser path
if(BUILD_SA_PLACE_LASER)
    add_subdirectory(src/c++/tools/sequentialadaboost)
    add_subdirectory(src/c++/tools/laserextractor)
    add_subdirectory(src/c++/components/LaserProcessor)
endif(BUILD_SA_PLACE_LASER)

# Visual path
if(BUILD_SA_PLACE_VISION)
    add_subdirectory(src/c++/tools/libcrfh)
    add_subdirectory(src/c++/components/VisualProcessor)
endif(BUILD_SA_PLACE_VISION)

# Check if at least one path is selected
if(NOT BUILD_SA_PLACE_LASER)
    if(NOT BUILD_SA_PLACE_VISION)
        message(FATAL_ERROR "Compiling at least one path (laser/vision) is mendatory for Place.SA")
    endif(NOT BUILD_SA_PLACE_VISION)
endif(NOT BUILD_SA_PLACE_LASER)

# Auxiliary components
if(BUILD_SA_PLACE_AUX)
    add_subdirectory(src/c++/components/Monitor)
    add_subdirectory(src/c++/components/PeekabotClient)
    add_subdirectory(src/c++/tools/dataeditor)
endif(BUILD_SA_PLACE_AUX)

# Testing
if(BUILD_SA_PLACE_TESTING)
#    add_subdirectory(src/c++/components/Tester)
endif(BUILD_SA_PLACE_TESTING)


