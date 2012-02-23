project(VideoGrabber)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

FIND_PACKAGE( OpenCV REQUIRED )
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

# Set the sources.
SET(SOURCES
   VideoGrabber.cpp
   )
include_directories(
   .
   ${VISUALIZATION_INCLUDE_DIRS}
   )
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

add_cast_component(VideoGrabber ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} 
   Video VideoUtils
   ${VISUALIZATION_LIBRARIES}
   ${OPENCV_LIBRARIES}
   )

