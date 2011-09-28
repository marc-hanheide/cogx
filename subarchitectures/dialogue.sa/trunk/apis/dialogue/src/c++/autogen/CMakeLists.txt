include(Slice2Cpp)

include_directories(.)

cast_slice2cpp(GEN_CPP GEN_HPP ref.ice ../../slice)

add_library(RefSlice SHARED ${GEN_CPP})
install(TARGETS RefSlice LIBRARY DESTINATION lib)
