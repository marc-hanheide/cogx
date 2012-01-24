include_directories(.)

add_cast_component(CategoricalDataSaver DataSaver.cpp)
link_cast_component(${CAST_COMPONENT_NAME} CategoricalData CategoricalShared)

