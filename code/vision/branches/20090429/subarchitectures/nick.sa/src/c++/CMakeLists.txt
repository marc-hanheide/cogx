include_directories(${COGX_ROOT}/tools/hardware/tts/src/c++)
include_directories(${COGX_ROOT}/tools/hardware/ptz/src/c++)
include_directories(${COGX_ROOT}/subarchitectures/nav.sa/src/c++)

add_cast_component(RandomGraphWalker RandomGraphWalker.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData TTS PTZ)
