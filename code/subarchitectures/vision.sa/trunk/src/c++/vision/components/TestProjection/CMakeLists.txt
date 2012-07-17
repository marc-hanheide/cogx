
include_directories(.)

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)


add_cast_component(TestProjection TestProjection.cpp)
link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils ${VISUALIZATION_LIBRARIES})

