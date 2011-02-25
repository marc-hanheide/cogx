include_directories(.)


include(Slice2Cpp)

cast_slice2cpp(GEN_CPP_M GEN_HPP_M manipulation.ice ${CAST_SLICE_DIR})
add_library(ManipulationData SHARED ${GEN_CPP_M})
target_link_libraries(ManipulationData CDL)
install(TARGETS ManipulationData LIBRARY DESTINATION lib)

