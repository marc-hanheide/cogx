include(Slice2Cpp)

include_directories(.)

cast_slice2cpp(GEN_CPP_M GEN_HPP_M 
   manipulation.ice
   ${CAST_SLICE_DIR} ${COGX_ROOT}/tools/math/src/slice)
cast_slice2cpp(GEN_CPP_MEXE GEN_HPP_MEXE
   manipulation_exe.ice 
   ${CAST_SLICE_DIR} ${COGX_ROOT}/tools/math/src/slice)

add_library(ManipulationData SHARED ${GEN_CPP_M} ${GEN_CPP_MEXE})
target_link_libraries(ManipulationData CDL Math)
install(TARGETS ManipulationData LIBRARY DESTINATION lib)

