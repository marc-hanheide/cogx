option(BUILD_SA_CONCEPTUAL_CHAIN_GRAPH_INFERENCER "Build Conceptual.SA ChainGraphInferencer component" YES)
if(BUILD_SA_CONCEPTUAL_CHAIN_GRAPH_INFERENCER)

    add_cast_component(ConceptualChainGraphInferencer ChainGraphInferencer.cc)
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData DefaultData ConceptualShared)
    set_target_properties(${CAST_COMPONENT_NAME} PROPERTIES INSTALL_RPATH ${ROCS_DIR}/lib)
    link_cast_component(${CAST_COMPONENT_NAME} dai)

endif(BUILD_SA_CONCEPTUAL_CHAIN_GRAPH_INFERENCER)

