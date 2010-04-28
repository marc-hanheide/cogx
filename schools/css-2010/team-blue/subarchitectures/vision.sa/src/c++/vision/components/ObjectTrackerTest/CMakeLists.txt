include_directories(.)
include_directories(../ObjectTracker)
include_directories(../ObjectTracker/Tracker/include)
include_directories(/usr/include/opencv)
include_directories(/usr/include/SDL)



# Set the sources.
SET(SOURCES ObjectTrackerTest.cpp)


add_cast_component(ObjectTrackerTest ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData ObjectTracker)

