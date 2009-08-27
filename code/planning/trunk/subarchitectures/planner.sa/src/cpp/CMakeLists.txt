include_directories(.)
include_directories(../../../fakebinder.sa/src/c++)
include_directories(../../../binder/src/c++)
include(Slice2Cpp)

cast_slice2cpp(GEN_CPP GEN_HPP Planner.ice ${CAST_SLICE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../../../binder/src/slice )

add_library(Planner SHARED ${GEN_CPP})
target_link_libraries(Planner BinderEssentials)
target_link_libraries(Planner CDL)
install(TARGETS Planner LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

add_cast_component(WMControl WMControl.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Planner)
