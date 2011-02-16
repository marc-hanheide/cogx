include_directories(.)
include_directories(../ObjectTracker)
include_directories(../ObjectTracker/Tracker/include)


# Set the sources.
SET(SOURCES VirtualSceneTest.cpp)


add_cast_component(VirtualSceneTest ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData ObjectTracker)

