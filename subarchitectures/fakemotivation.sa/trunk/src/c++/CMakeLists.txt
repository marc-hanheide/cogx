include_directories(${COGX_ROOT}/subarchitectures/planner.sa/src/c++)
include_directories(${COGX_ROOT}/tools/beliefs/src/c++)

add_cast_component(FakeMotivationSA FakeMotivationSA.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Planner)
