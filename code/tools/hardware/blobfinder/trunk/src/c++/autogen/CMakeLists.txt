include_directories(.)
include(Slice2Cpp)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP PTZ.ice ${CAST_SLICE_DIR})
add_library(PTZ SHARED ${GEN_CPP})
target_link_libraries(PTZ CDL ${ICE_LIBS})
install(TARGETS PTZ LIBRARY DESTINATION lib)


