include_directories(.)


include(Slice2Cpp)

cast_slice2cpp(GEN_CPP_M GEN_HPP_M manipulation.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/tools/math/src/slice
    ${COGX_ROOT}/tools/hardware/video/src/slice
    ${COGX_ROOT}/subarchitectures/vision.sa/src/slice
    )
add_library(ManipulationData SHARED ${GEN_CPP_M})
target_link_libraries(ManipulationData CDL)
install(TARGETS ManipulationData LIBRARY DESTINATION lib)

