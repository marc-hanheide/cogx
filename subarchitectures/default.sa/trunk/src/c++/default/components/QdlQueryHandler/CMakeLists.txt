# Components
option(BUILD_SA_DEFAULT_QDL_QUERY_HANDLER "Build Default.SA QdlQueryHandler component" YES)
if(BUILD_SA_DEFAULT_QDL_QUERY_HANDLER)
add_cast_component(QdlQueryHandler QdlQueryHandler.cc)
#link_cast_component(${CAST_COMPONENT_NAME} )
endif(BUILD_SA_DEFAULT_QDL_QUERY_HANDLER)

