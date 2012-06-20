project (Kinect)
include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)

include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)

include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/components)

# auto generated code
add_subdirectory(src/c++/autogen)
if (DO_SLICE_ONLY)
   return()
endif()


include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen/)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/components/)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_subdirectory(src/c++/components)

