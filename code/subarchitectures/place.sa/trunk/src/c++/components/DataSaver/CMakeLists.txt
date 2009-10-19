include_directories(.)

add_cast_component(PlaceDataSaver DataSaver.cpp)
link_cast_component(${CAST_COMPONENT_NAME} PlaceData PlaceShared)
