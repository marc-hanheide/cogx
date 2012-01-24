include(Slice2Cpp)

include_directories(.)

cast_slice2cpp(GEN_CPP_M GEN_HPP_M manipulation.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/tools/math/src/slice)
add_library(ManipulationData SHARED ${GEN_CPP_M})
target_link_libraries(ManipulationData CDL Math)
install(TARGETS ManipulationData LIBRARY DESTINATION lib)
