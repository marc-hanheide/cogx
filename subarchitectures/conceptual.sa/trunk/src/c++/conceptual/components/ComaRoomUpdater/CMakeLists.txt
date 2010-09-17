option(BUILD_SA_CONCEPTUAL_BELIEF_UPDATER "Build Conceptual.SA BeliefUpdater component" YES)
if(BUILD_SA_CONCEPTUAL_BELIEF_UPDATER)

    add_cast_component(ConceptualBeliefUpdater BeliefUpdater.cc)
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData)

endif(BUILD_SA_CONCEPTUAL_BELIEF_UPDATER)

