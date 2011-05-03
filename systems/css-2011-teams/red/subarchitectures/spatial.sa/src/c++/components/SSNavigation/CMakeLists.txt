
include_directories(.)

find_package(OpenCV REQUIRED)

include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/autogen)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)
#include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/autogen)

#include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/TomGine/TomGine)

add_cast_component(SSNavigation SSNavigation.cpp)
#link_cast_component(${CAST_COMPONENT_NAME} freetype ftgl GLWindow TomGine TomGineWraper)
#link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)
link_cast_component(${CAST_COMPONENT_NAME} VisionData)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS})
