# Components
option(BUILD_SA_DEFAULT_DEFAULT_KNOWLEDGE_LOADER "Build Default.SA DefaultKnowledgeLoader component" YES)
if(BUILD_SA_DEFAULT_DEFAULT_KNOWLEDGE_LOADER)
add_cast_component(DefaultKnowledgeLoader DefaultKnowledgeLoader.cc)
#link_cast_component(${CAST_COMPONENT_NAME} )
endif(BUILD_SA_DEFAULT_DEFAULT_KNOWLEDGE_LOADER)

