include_directories(../../../planner.sa/src/c++)

add_cast_component(FakeMotivationSA FakeMotivationSA.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Planner)
