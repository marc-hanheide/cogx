option(BUILD_SA_CONCEPTUAL_QDL_QUERY_HANDLER "Build Conceptual.SA QdlQueryHandler component" YES)
if(BUILD_SA_CONCEPTUAL_QDL_QUERY_HANDLER)

    add_cast_component(ConceptualQdlQueryHandler ConceptualQdlQueryHandler.cc)
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData)

endif(BUILD_SA_CONCEPTUAL_QDL_QUERY_HANDLER)

