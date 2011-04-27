include_directories(.)
include_directories(../ObjectTracker)
include_directories(../ObjectTracker/Tracker/include)
include_directories(/usr/include/SDL)



# Set the sources.
SET(SOURCES ObjectTrackerDriver.cpp)


add_cast_component(ObjectTrackerDriver ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData ObjectTracker)

