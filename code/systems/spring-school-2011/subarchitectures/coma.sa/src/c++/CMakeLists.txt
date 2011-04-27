include(Slice2Cpp)

include_directories(.)
include_directories(${COGX_ROOT}/subarchitectures/spatial.sa/src/c++/autogen)

cast_slice2cpp(GEN_CPP GEN_HPP ComaData.ice ${CAST_SLICE_DIR}
	       ${COGX_ROOT}/subarchitectures/spatial.sa/src/slice )
add_library(ComaData SHARED ${GEN_CPP})
target_link_libraries(ComaData CDL SpatialProbabilities)
install(TARGETS ComaData LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
