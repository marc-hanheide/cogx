include_directories(${COGX_ROOT}/subarchitectures/planner.sa/src/c++)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs/src/cpp/autogen)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs+cast/src/cpp/autogen)

add_cast_component(FakeMotivationSA FakeMotivationSA.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Planner)
