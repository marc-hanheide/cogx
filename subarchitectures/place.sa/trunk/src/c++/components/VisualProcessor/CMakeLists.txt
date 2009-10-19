include_directories(.)

add_cast_component(PlaceVisualProcessor VisualProcessor.cpp)
link_cast_component(${CAST_COMPONENT_NAME} PlaceData PlaceShared)
link_cast_component(${CAST_COMPONENT_NAME} crfh svm-mod)

