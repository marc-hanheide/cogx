include_directories(.)
include_directories(../../../fakebinder.sa/src/c++)
include(Slice2Cpp)

cast_slice2cpp(GEN_CPP GEN_HPP Planner.ice ${CAST_SLICE_DIR})
add_library(Planner SHARED ${GEN_CPP})
install(TARGETS Planner LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

add_cast_component(WMControl WMControl.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Planner FakeBinderData)
