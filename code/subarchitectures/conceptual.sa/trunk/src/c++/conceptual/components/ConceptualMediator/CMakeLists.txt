option(BUILD_SA_CONCEPTUAL_MEDIATOR "Build Conceptual.SA Mediator component" YES)
if(BUILD_SA_CONCEPTUAL_MEDIATOR)

    include_directories(${COGX_ROOT}/subarchitectures/binder/src/c++/autogen)
    include_directories(${COGX_ROOT}/subarchitectures/binder/src/c++/)
    include_directories(${COGX_ROOT}/tools/beliefmodels/src/c++/autogen)

    add_cast_component(ConceptualMediator ConceptualMediator.cc)
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData BinderAbstract BeliefModels)

endif(BUILD_SA_CONCEPTUAL_MEDIATOR)

