option(BUILD_SA_CONCEPTUAL_OBSERVER "Build Conceptual.SA Observer component" YES)
if(BUILD_SA_CONCEPTUAL_OBSERVER)

    add_cast_component(ConceptualObserver ConceptualObserver.cc)
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData)

endif(BUILD_SA_CONCEPTUAL_OBSERVER)

