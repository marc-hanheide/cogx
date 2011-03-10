include_directories(../ObjectTracker/Tracker/include)


include_directories(
   .
   ${VISUALIZATION_INCLUDE_DIRS}
   )

include_directories(ARToolKit/include)

# Set the sources.
SET(SOURCES AR_Pose.cpp object.cpp)

add_cast_component(AR_Pose ${SOURCES})

link_cast_component(${CAST_COMPONENT_NAME} 
   Math AR VisionData Video VideoUtils   ${VISUALIZATION_LIBRARIES}
   ${OPENCV_LDFLAGS}
   )