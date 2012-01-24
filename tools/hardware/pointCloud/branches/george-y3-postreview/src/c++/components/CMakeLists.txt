find_package(OpenCV REQUIRED)

include_directories(.)
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${OPENCV_CFLAGS})

link_directories (/usr/local/lib)
link_directories (/usr/local/cuda/lib)
link_directories (/usr/local/cuda/sdk/lib)
link_directories (/usr/local/cuda/sdk/C/lib)
link_directories (${COGX_ROOT}/tools/hardware/pointCloud/src/c++/components/gpustereo)

SET(CMAKE_CXX_FLAGS ${CXX_FLAGS} "-fPIC")

# GPU LIBRARY for STEREO SERVER
option(BUILD_HAL_STEREO_GPU "Build the GPU Library for Stereo Server" NO)
if(BUILD_HAL_STEREO_GPU)
  add_subdirectory(gpustereo)
  link_directories (${CMAKE_CURRENT_BINARY_DIR}/gpustereo)
  SET(CMAKE_CXX_FLAGS ${CXX_FLAGS} "-DHAVE_GPU_STEREO")
endif(BUILD_HAL_STEREO_GPU)

# POINT CLOUD SERVER
add_cast_component(PointCloudServer PointCloudServer.cpp StereoCamera.cpp)
link_cast_component(${CAST_COMPONENT_NAME} CASTCore CDL CASTArchitecture)
link_cast_component(${CAST_COMPONENT_NAME} PointCloud Video VideoUtils PTZ)

# STEREO SERVER
add_cast_component(StereoServer StereoServer.cpp)
link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudServer Video VideoUtils)

if(BUILD_HAL_STEREO_GPU)
 link_cast_component(${CAST_COMPONENT_NAME} gpustereo)
 link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS} cudart cutil)
 add_dependencies(StereoServer GpuStereo)
endif(BUILD_HAL_STEREO_GPU)

# POINT CLOUD CLIENT
add_cast_component(PointCloudClient PointCloudClient.cpp)
link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudServer Video CASTCore CDL)


