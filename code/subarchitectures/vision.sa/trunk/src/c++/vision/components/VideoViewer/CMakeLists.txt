FIND_PACKAGE( OpenCV REQUIRED )

# Set the sources.
SET(SOURCES VideoViewer.cpp)
include_directories(
   .
   ${VISUALIZATION_INCLUDE_DIRS}
   )
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

add_cast_component(VideoViewer ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} 
   VisionData Video VideoUtils
   ${VISUALIZATION_LIBRARIES}
   ${OpenCV_LIBS}
   )

