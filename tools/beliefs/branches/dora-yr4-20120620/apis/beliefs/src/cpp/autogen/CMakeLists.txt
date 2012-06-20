include(Slice2Cpp)

include_directories(.)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP beliefs.ice ../../slice)

add_library(Beliefs SHARED ${GEN_CPP})
install(TARGETS Beliefs LIBRARY DESTINATION lib)

