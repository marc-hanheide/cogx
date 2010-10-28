option(BUILD_SA_DEFAULT_QUERY_HANDLER "Build Default.SA QueryHandler component" YES)
if(BUILD_SA_DEFAULT_QUERY_HANDLER)

    add_cast_component(DefaultQueryHandler QueryHandler.cc)
    link_cast_component(${CAST_COMPONENT_NAME} DefaultData)

endif(BUILD_SA_DEFAULT_QUERY_HANDLER)

