option(BUILD_SA_SPATIAL_PLACE_PROPERTY_SAVER "Build Spatial.SA PlacePropertSaver component" YES)
if(BUILD_SA_SPATIAL_PLACE_PROPERTY_SAVER)

  add_cast_component(PlacePropertySaver PlacePropertySaver.cpp)
  link_cast_component(${CAST_COMPONENT_NAME} SpatialProperties CASTUtils)

endif(BUILD_SA_SPATIAL_PLACE_PROPERTY_SAVER)
