include_directories(.)

add_cast_component(PlaceCueIntegrator CueIntegrator.cpp)
link_cast_component(${CAST_COMPONENT_NAME} PlaceData PlaceShared)
link_cast_component(${CAST_COMPONENT_NAME} svm-mod)

