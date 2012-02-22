project (PointCloud)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/ptz/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)

# auto generated code
add_subdirectory(src/c++/autogen)
if (DO_SLICE_ONLY)
   return()
endif()

add_subdirectory(src/c++/components)

