include_directories(./autogen)
include_directories(.)

add_cast_component(MultiSATester MultiSATester.cpp)
link_cast_component(${CAST_COMPONENT_NAME} TestingData )
