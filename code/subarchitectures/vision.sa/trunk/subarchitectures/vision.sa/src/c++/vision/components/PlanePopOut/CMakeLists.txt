include_directories(.)

add_cast_component(PlanePopOut PlanePopOut.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Stereo StereoClient)
link_cast_component(${CAST_COMPONENT_NAME} GLU glut)
link_cast_component(${CAST_COMPONENT_NAME} VisionData VideoUtils)


