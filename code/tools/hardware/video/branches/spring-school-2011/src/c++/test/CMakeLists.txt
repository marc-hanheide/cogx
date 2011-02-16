include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components)

add_cast_component(TestVideoClient TestVideoClient.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Video)

