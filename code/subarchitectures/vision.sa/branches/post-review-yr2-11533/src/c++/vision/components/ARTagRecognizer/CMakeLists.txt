include_directories(../ObjectTracker/Tracker/include)


include_directories(
   .
   ${VISUALIZATION_INCLUDE_DIRS}
   )

include_directories(ARToolKit/include)

# Set the sources.
SET(SOURCES ARTagRecognizer.cpp)

add_cast_component(ARTagRecognizer ${SOURCES})

link_cast_component(${CAST_COMPONENT_NAME} 
   Math AR VisionData Video VideoUtils   ${VISUALIZATION_LIBRARIES}
   ${OPENCV_LDFLAGS}
   )
