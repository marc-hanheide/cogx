include_directories(.)

include_directories(${VISUALIZATION_INCLUDE_DIRS})
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

# Set the sources.
SET(SOURCES SOIFilter.cpp GCoptimization.cpp graph.cpp LinkedBlockList.cpp maxflow.cpp)

add_cast_component(SOIFilter ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME}
   VisionData VideoUtils
   Stereo StereoClient
   ${VISUALIZATION_LIBRARIES}
   )
