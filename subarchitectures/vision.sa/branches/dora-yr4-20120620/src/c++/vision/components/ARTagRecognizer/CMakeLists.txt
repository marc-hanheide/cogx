include_directories(../ObjectTracker/Tracker/include)

ENABLE_COGX_VISUALIZATION(TRUE) # Really?
 
# Set the sources.
SET(SOURCES ARTagRecognizer.cpp)
include_directories(.)

include_directories(ARToolKit/include)
link_directories(ARToolKit/lib)
add_cast_component(ARTagRecognizer ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} 
   Math AR VisionData Video VideoUtils
   ${VISUALIZATION_LIBRARIES}
   ${OPENCV_LDFLAGS}
   )

