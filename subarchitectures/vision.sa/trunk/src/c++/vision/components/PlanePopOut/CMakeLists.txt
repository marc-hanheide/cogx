include_directories(.)
include_directories(${COGX_ROOT}/subarchitectures/nav.sa/src/c++/autogen)

add_cast_component(PlanePopOut PlanePopOut.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Stereo StereoClient)
link_cast_component(${CAST_COMPONENT_NAME} GLU glut)
link_cast_component(${CAST_COMPONENT_NAME} VisionData VideoUtils NavData)


