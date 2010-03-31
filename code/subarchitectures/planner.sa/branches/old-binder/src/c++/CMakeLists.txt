include_directories(.)
#include_directories(../../../fakebinder.sa/src/c++)
#include_directories(../../../binder/src/c++)
include_directories(${COGX_ROOT}/subarchitectures/binder/src/c++/autogen)
include(Slice2Cpp)

cast_slice2cpp(GEN_CPP GEN_HPP Planner.ice ${CAST_SLICE_DIR} ${COGX_ROOT}/subarchitectures/binder/src/slice )

add_library(Planner SHARED ${GEN_CPP})
target_link_libraries(Planner BinderEssentials)
target_link_libraries(Planner CDL)

find_package(Boost 1.35.0 REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

install(TARGETS Planner LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

add_cast_component(WMControl WMControl.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Planner)
link_cast_component(${CAST_COMPONENT_NAME} ${Boost_LIBRARIES})
