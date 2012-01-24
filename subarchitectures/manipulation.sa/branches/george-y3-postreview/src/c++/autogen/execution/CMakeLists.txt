include(Slice2Cpp)

include_directories(.)

cast_slice2cpp(GEN_CPP_M GEN_HPP_M manipulation_exe.ice ${CAST_SLICE_DIR}
	${COGX_ROOT}/tools/math/src/slice)

add_library(ManipulationExeData SHARED ${GEN_CPP_M})
target_link_libraries(ManipulationExeData CDL Math)
install(TARGETS ManipulationExeData LIBRARY DESTINATION lib)
