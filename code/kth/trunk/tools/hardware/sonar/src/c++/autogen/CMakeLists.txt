include_directories(.)
include(Slice2Cpp)

# expects to find .ice file under PROJECT/src/slice e.g. sonar/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP Sonar.ice ${CAST_SLICE_DIR})
add_library(Sonar SHARED ${GEN_CPP})
target_link_libraries(Sonar CDL Ice IceUtil)
install(TARGETS Sonar LIBRARY DESTINATION lib)


