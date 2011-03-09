include_directories(../ObjectTracker/Tracker/include)

# Set the sources.
SET(SOURCES ARTagRecognizer.cpp)
include_directories(
   .
   ${VISUALIZATION_INCLUDE_DIRS}
   )

  
include_directories(ARToolKit/include)
link_directories(ARToolKit/lib)
#add_definitions(-DFEAT_VISUALIZATION_OPENCV)
add_cast_component(ARTagRecognizer ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} 
   Math AR VisionData Video VideoUtils
   ${VISUALIZATION_LIBRARIES}
   ${OPENCV_LDFLAGS}
   )

