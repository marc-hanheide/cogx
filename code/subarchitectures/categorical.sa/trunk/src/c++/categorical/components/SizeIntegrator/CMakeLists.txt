include_directories(.)

add_cast_component(CategoricalSizeIntegrator SizeIntegrator.cpp OutputsCache.cpp)
link_cast_component(${CAST_COMPONENT_NAME} CategoricalData CategoricalShared FrontierInterface SpatialProperties)

