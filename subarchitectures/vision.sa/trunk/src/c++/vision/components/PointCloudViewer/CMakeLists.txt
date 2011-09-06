
include_directories(.)

find_package(OpenCV REQUIRED)

include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/TomGine)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/TomGine/TomGine)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen)


add_cast_component(PointCloudViewer PointCloudViewer.cpp)
link_cast_component(${CAST_COMPONENT_NAME} freetype ftgl GLWindow TomGine TomGineWraper)
link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)


