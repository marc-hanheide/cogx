option(BUILD_SA_CONCEPTUAL_TESTER "Build Conceptual.SA Tester component" YES)
if(BUILD_SA_CONCEPTUAL_TESTER)

    add_cast_component(ConceptualTester Tester.cc)
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData)

endif(BUILD_SA_CONCEPTUAL_TESTER)

