include_directories(.)
include(${CAST_ROOT}/cmake/Slice2Cpp.cmake)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP Laser.ice ${CAST_SLICE})
add_library(Laser SHARED ${GEN_CPP})
target_link_libraries(Laser CDL Ice IceUtil)
install(TARGETS Laser LIBRARY DESTINATION lib)


