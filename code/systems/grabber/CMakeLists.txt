cmake_minimum_required(VERSION 2.8)

include(cmake/CMakeLists.txt)

if (DO_SLICE_ONLY)
   return()
endif()

# From vision.sa CMakeLists.txt
include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/castutils/src/c++)

add_subdirectory(components/CameraMount)
add_subdirectory(components/VideoGrabber)
add_subdirectory(components/VideoViewer)
