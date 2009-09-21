include(Slice2Cpp)

cast_slice2cpp(GEN_CPP GEN_HPP MatlabData.ice ${CAST_SLICE_DIR})

add_library(MatlabData SHARED ${GEN_CPP})

install(TARGETS MatlabData LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

