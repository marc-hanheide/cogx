cmake_minimum_required(VERSION 2.8)

project (VisualisationSa)

if (DEFINED COGX_ROOT)
   # Build as a part of the CogX system
   set (HAS_COGX_MATH TRUE)
   set (COGX_MATH_ROOT ${COGX_ROOT})
   set (HAS_COGX_VIDEO TRUE)
   set (COGX_VIDEO_ROOT ${COGX_ROOT})
else()
   # Standalone build
   include(cmake/cast.cmake)
   option(HAS_COGX_MATH "CogX Math library is available" OFF)
   option(HAS_COGX_VIDEO "CogX Video library is available" OFF)
   set (COGX_MATH_ROOT ..)
   set (COGX_VIDEO_ROOT ..)
endif()

if (HAS_COGX_MATH)
   include_directories(${COGX_MATH_ROOT}/tools/math/src/c++/autogen)
   include_directories(${COGX_MATH_ROOT}/tools/math/src/c++/math)
   add_definitions(-DHAVE_COGX_MATH)
endif()
if (HAS_COGX_VIDEO)
   include_directories(${COGX_VIDEO_ROOT}/tools/hardware/video/src/c++/utils)
   include_directories(${COGX_VIDEO_ROOT}/tools/hardware/video/src/c++/components)
   include_directories(${COGX_VIDEO_ROOT}/tools/hardware/video/src/c++/autogen)
   add_definitions(-DHAVE_COGX_VIDEO)
endif()

# auto generated code
include_directories(src/c++/autogen)
include_directories(src/c++/core)

add_subdirectory(src/c++/autogen)
if (DO_SLICE_ONLY)
   return()
endif()

if(BUILD_PYTHON_COMPONENTS)
   add_subdirectory(src/python)
endif(BUILD_PYTHON_COMPONENTS)

option(BUILD_SA_V11N_AUTOGEN_ONLY "ONLY build autogen parts of V11N" OFF)
if(BUILD_SA_V11N_AUTOGEN_ONLY)
   return()
endif()

include(cmake/v11n-private.cmake)
add_subdirectory(src/c++/core)
add_subdirectory(src/c++/clients)

