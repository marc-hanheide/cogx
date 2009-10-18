include_directories(.)

add_cast_component(PlaceController Controller.cpp)
link_cast_component(${CAST_COMPONENT_NAME} PlaceData PlaceShared)

