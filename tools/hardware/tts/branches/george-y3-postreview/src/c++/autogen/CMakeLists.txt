include_directories(.)
include(Slice2Cpp)

# expects to find .ice file under PROJECT/src/slice e.g. tts/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP TTS.ice ${CAST_SLICE})
add_library(TTS SHARED ${GEN_CPP})
target_link_libraries(TTS CDL ${ICE_LIBS})
install(TARGETS TTS LIBRARY DESTINATION lib)


