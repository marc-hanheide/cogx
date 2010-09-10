option(BUILD_SA_DEFAULT_DEFAULT_KNOWLEDGE_LOADER "Build Default.SA KnowledgeLoader component" YES)
if(BUILD_SA_DEFAULT_DEFAULT_KNOWLEDGE_LOADER)

    add_cast_component(DefaultKnowledgeLoader DefaultKnowledgeLoader.cc)
    link_cast_component(${CAST_COMPONENT_NAME} DefaultData)

endif(BUILD_SA_DEFAULT_DEFAULT_KNOWLEDGE_LOADER)

