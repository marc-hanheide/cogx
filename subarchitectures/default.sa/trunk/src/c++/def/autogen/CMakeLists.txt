include(Slice2Cpp)
include_directories(.)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP_SD GEN_HPP_SD DefaultData.ice ${CAST_SLICE_DIR}
	       ${COGX_ROOT}/subarchitectures/spatial.sa/src/slice)
add_library(DefaultData SHARED ${GEN_CPP_SD})
target_link_libraries(DefaultData CDL SpatialProbabilities)
install(TARGETS DefaultData LIBRARY DESTINATION lib)

