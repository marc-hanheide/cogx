include(Slice2Cpp)

include_directories(.)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP abducer.ice ../../slice)

add_library(AbducerIce SHARED ${GEN_CPP})
install(TARGETS AbducerIce LIBRARY DESTINATION lib)

