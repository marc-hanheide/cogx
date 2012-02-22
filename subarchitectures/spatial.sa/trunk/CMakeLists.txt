cmake_minimum_required (VERSION 2.6)
project (SPATIALSA)

# Set build type
# set(CMAKE_BUILD_TYPE Release)

# You can prepare verbose Makefile to see what's going.
set(CMAKE_VERBOSE_MAKEFILE true)

# Include dirs
include_directories(~/projects/rocs-toolkit/david/src)
include_directories(${COGX_ROOT}/subarchitectures/spatial.sa/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/robotbase/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)

if(BUILD_SA_BINDING)
  include_directories(${COGX_ROOT}/subarchitectures/binder/src/c++/autogen)
endif(BUILD_SA_BINDING)
include_directories(${COGX_ROOT}/tools/d-lib/src/c++/d)
include_directories(src/c++)
include_directories(src/c++/autogen)



# -----------------------------------------------------------------------
# Subdirs
# -----------------------------------------------------------------------

# Auto generated code
add_subdirectory (src/c++/autogen)
if (DO_SLICE_ONLY)
   return()
endif()
#
# Remove warnings about deprecated libs.
add_definitions(-Wno-deprecated)


find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})


# Mendatory components
add_subdirectory(src/c++/)

# Options
#option(BUILD_SA_SPATIAL_XXX "Build Spatial.SA XXX" YES)

# Laser path
#if(BUILD_SA_SPATIAL_XXX)
#    add_subdirectory(src/c++/components/XXX)
#endif(BUILD_SA_SPATIAL_XXX)



