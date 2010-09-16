option(BUILD_SA_CONCEPTUAL_QUERY_HANDLER "Build Conceptual.SA QueryHandler component" YES)
if(BUILD_SA_CONCEPTUAL_QUERY_HANDLER)

    add_cast_component(ConceptualQueryHandler ConceptualQueryHandler.cc)
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData)

endif(BUILD_SA_CONCEPTUAL_QUERY_HANDLER)

