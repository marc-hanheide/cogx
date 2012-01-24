include_directories(.)

add_cast_component(ViewConeMoveCostGenerator ViewConeMoveCostGenerator.hpp ViewConeMoveCostGenerator.cpp)
link_cast_component(${CAST_COMPONENT_NAME} VisionData)


