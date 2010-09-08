# Components
option(BUILD_SA_DEFAULT_CHAIN_GRAPH_INFERENCER "Build Default.SA ChainGraphInferencer component" YES)
if(BUILD_SA_DEFAULT_CHAIN_GRAPH_INFERENCER)
add_cast_component(ChainGraphInferencer ChainGraphInferencer.cc)
#link_cast_component(${CAST_COMPONENT_NAME} )
endif(BUILD_SA_DEFAULT_CHAIN_GRAPH_INFERENCER)

