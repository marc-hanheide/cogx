option(BUILD_SA_DEFAULT_CHAIN_GRAPH_INFERENCER "Build Default.SA ChainGraphInferencer component" YES)
if(BUILD_SA_DEFAULT_CHAIN_GRAPH_INFERENCER)

    add_cast_component(DefaultChainGraphInferencer ChainGraphInferencer.cc)
    link_cast_component(${CAST_COMPONENT_NAME} DefaultData)

endif(BUILD_SA_DEFAULT_CHAIN_GRAPH_INFERENCER)

