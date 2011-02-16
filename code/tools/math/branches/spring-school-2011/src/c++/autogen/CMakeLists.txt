include(Slice2Cpp)

cast_slice2cpp(GEN_CPP GEN_HPP Math.ice)

add_library(Math SHARED ${GEN_CPP})

install(TARGETS Math LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

