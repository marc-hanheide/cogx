option(BUILD_SA_CONCEPTUAL_OBSERVER "Build Conceptual.SA Observer component" YES)
if(BUILD_SA_CONCEPTUAL_OBSERVER)

    add_cast_component(ConceptualObserver Observer.cc)
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData ComaData SpatialData SpatialProperties)

endif(BUILD_SA_CONCEPTUAL_OBSERVER)

