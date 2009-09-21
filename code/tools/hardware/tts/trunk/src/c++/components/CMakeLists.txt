include_directories(..)

add_cast_component(TTSServerDoubleTalk TTSServerDoubleTalk.cpp)
link_cast_component(${CAST_COMPONENT_NAME} TTS)

