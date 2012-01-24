option(BUILD_SA_DEFAULT_TESTER "Build Default.SA Tester component" YES)
if(BUILD_SA_DEFAULT_TESTER)

    add_cast_component(DefaultTester ${SOURCES} Tester.cc)
    link_cast_component(${CAST_COMPONENT_NAME} DefaultData ComaData)

endif(BUILD_SA_DEFAULT_TESTER)

