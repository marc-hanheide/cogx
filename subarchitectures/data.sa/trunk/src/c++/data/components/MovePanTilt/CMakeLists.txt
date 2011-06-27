include_directories(.)
include_directories(${COGX_ROOT}/tools/hardware/ptz/src/c++/autogen)

add_cast_component(CameraMountTest CameraMountTest.cpp)
link_cast_component(${CAST_COMPONENT_NAME} PTZ)

