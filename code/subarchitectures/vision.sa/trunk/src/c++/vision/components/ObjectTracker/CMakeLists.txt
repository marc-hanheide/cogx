project(ObjectTracker)

FIND_PACKAGE( OpenCV REQUIRED )

include_directories(.)
include_directories(Tracker/include)
include_directories(/usr/include/SDL)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen)

add_subdirectory(Tracker)

link_directories(/usr/local/lib)

# Set the sources
SET(SOURCES ObjectTracker.cpp)

add_cast_component(ObjectTracker ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} Tracker)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils PointCloudClient)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS} SDL GLU glut)
