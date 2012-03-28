option(BUILD_SA_SPATIAL_PLACE_PROPERTY_SAVER "Build Spatial.SA PlacePropertSaver component" YES)
if(BUILD_SA_SPATIAL_PLACE_PROPERTY_SAVER)

  FIND_PACKAGE (Boost 1.38.0 REQUIRED COMPONENTS filesystem)
  set (BOOST_LIBRARIES ${Boost_LIBRARIES})

  add_cast_component(PlacePropertySaver PlacePropertySaver.cpp)
  link_cast_component(${CAST_COMPONENT_NAME} SpatialProperties CASTUtils ${BOOST_LIBRARIES})

endif(BUILD_SA_SPATIAL_PLACE_PROPERTY_SAVER)
