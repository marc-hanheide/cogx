include_directories(.)
#include_directories(../ObjectTracker)
#include_directories(../ObjectTracker/include)
#include_directories(/usr/include/SDL)



# Set the sources.
SET(SOURCES StereoFlapDetectorDriver.cpp)


add_cast_component(StereoFlapDetectorDriver ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData StereoFlapDetector)

