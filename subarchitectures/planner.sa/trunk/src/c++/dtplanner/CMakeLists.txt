include_directories(.)

add_cast_component(dtp-control dtp-control.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Planner)
link_cast_component(${CAST_COMPONENT_NAME} ${Boost_LIBRARIES})
