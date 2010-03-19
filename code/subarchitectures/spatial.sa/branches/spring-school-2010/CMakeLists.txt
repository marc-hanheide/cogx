cmake_minimum_required (VERSION 2.6)
project (SPATIALSA)

# Set build type
# set(CMAKE_BUILD_TYPE Release)

# You can prepare verbose Makefile to see what's going.
set(CMAKE_VERBOSE_MAKEFILE true)

option(BUILD_SPATIAL4BINDING "Build spatial monitors for binder" NO)


# Include dirs
include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/robotbase/src/c++/autogen)
if(BUILD_SPATIAL4BINDING)
  include_directories(${COGX_ROOT}/subarchitectures/binder/src/c++/autogen)
endif(BUILD_SPATIAL4BINDING)
include_directories(${COGX_ROOT}/tools/d-lib/src/c++/d)
include_directories(src/c++)
include_directories(src/c++/autogen)



# -----------------------------------------------------------------------
# Subdirs
# -----------------------------------------------------------------------

# Auto generated code
add_subdirectory (src/c++/autogen)

# Mendatory components
add_subdirectory(src/c++/)

# Options
#option(BUILD_SA_SPATIAL_XXX "Build Spatial.SA XXX" YES)

# Laser path
#if(BUILD_SA_SPATIAL_XXX)
#    add_subdirectory(src/c++/components/XXX)
#endif(BUILD_SA_SPATIAL_XXX)



