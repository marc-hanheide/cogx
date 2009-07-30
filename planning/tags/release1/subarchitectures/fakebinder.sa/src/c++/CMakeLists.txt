include_directories(.)
include(Slice2Cpp)

cast_slice2cpp(GEN_CPP GEN_HPP FakeBinderData.ice ${CAST_SLICE_DIR})
add_library(FakeBinderData SHARED ${GEN_CPP})
install(TARGETS FakeBinderData LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

add_cast_component(FakeBinderSA FakeBinderSA.cpp)
link_cast_component(${CAST_COMPONENT_NAME} FakeBinderData)
