option(BUILD_SA_DEFAULT_CHAIN_GRAPH_INFERENCER "Build Default.SA ChainGraphInferencer component" YES)
if(BUILD_SA_DEFAULT_CHAIN_GRAPH_INFERENCER)

    add_cast_component(DefaultChainGraphInferencer ChainGraphInferencer.cc)
    link_cast_component(${CAST_COMPONENT_NAME} DefaultData ComaData SpatialData ConceptualShared)
    set_target_properties(${CAST_COMPONENT_NAME} PROPERTIES INSTALL_RPATH ${ROCS_DIR}/lib)
    link_cast_component(${CAST_COMPONENT_NAME} rocs_core)

endif(BUILD_SA_DEFAULT_CHAIN_GRAPH_INFERENCER)

