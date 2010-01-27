include_directories(.)
include_directories(../ObjectTracker)
include_directories(../ObjectTracker/Tracker/include)
include_directories(/usr/include/opencv)

# Set the sources.
SET(SOURCES ObjectRecognizer3D.cpp)


add_cast_component(ObjectRecognizer3D ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData ObjectTracker)

