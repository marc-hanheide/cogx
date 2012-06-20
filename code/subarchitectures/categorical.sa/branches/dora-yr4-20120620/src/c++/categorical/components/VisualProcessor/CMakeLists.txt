include_directories(.)

add_cast_component(CategoricalVisualProcessor VisualProcessor.cpp)
link_cast_component(${CAST_COMPONENT_NAME} CategoricalData CategoricalShared)
link_cast_component(${CAST_COMPONENT_NAME} crfh svm-mod)

