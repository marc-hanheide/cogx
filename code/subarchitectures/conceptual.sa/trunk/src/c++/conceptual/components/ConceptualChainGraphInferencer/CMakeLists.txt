option(BUILD_SA_CONCEPTUAL_CHAIN_GRAPH_INFERENCER "Build Conceptual.SA ChainGraphInferencer component" YES)
if(BUILD_SA_CONCEPTUAL_CHAIN_GRAPH_INFERENCER)

    add_cast_component(ConceptualChainGraphInferencer ConceptualChainGraphInferencer.cc)
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData)

endif(BUILD_SA_CONCEPTUAL_CHAIN_GRAPH_INFERENCER)

