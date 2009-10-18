include_directories(.)
include(Slice2Cpp)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP PlaceData.ice ${CAST_SLICE_DIR}
		${COGX_ROOT}/tools/hardware/laser/src/slice
		${COGX_ROOT}/tools/hardware/robotbase/src/slice
		${COGX_ROOT}/tools/math/src/slice
		${COGX_ROOT}/tools/hardware/video/src/slice
		)
add_library(PlaceData SHARED ${GEN_CPP})
target_link_libraries(PlaceData CDL Laser Robotbase Video)
install(TARGETS PlaceData LIBRARY DESTINATION lib)



