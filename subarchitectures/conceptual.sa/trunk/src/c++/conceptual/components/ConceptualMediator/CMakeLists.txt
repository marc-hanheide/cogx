option(BUILD_SA_DEFAULT_QDL_QUERY_HANDLER "Build Default.SA QdlQueryHandler component" YES)
if(BUILD_SA_DEFAULT_QDL_QUERY_HANDLER)

    include_directories(${COGX_ROOT}/subarchitectures/binder/src/c++/autogen)
    include_directories(${COGX_ROOT}/subarchitectures/binder/src/c++/)
    include_directories(${COGX_ROOT}/tools/beliefmodels/src/c++/autogen)

    add_cast_component(QdlQueryHandler QdlQueryHandler.cc)
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData BinderAbstract BeliefModels)

endif(BUILD_SA_DEFAULT_QDL_QUERY_HANDLER)

