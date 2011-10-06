include_directories(.)
include_directories(${COGX_ROOT}/tools/hardware/ptz/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/castutils/src/c++)

add_cast_component(CameraMount CameraMount.cpp)
link_cast_component(${CAST_COMPONENT_NAME} PTZ Video VideoUtils)

add_cast_component(CameraMountTest CameraMountTest.cpp)
link_cast_component(${CAST_COMPONENT_NAME} PTZ Video)

