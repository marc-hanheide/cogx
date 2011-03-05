option(BUILD_SA_CONCEPTUAL_TESTER "Build Conceptual.SA Tester component" YES)
if(BUILD_SA_CONCEPTUAL_TESTER)

    qt4_add_resources( RC_SOURCES resources.qrc )
    qt4_wrap_ui( UI_HEADERS MainDialog.ui ObjectPlacePropertyDialog.ui ObjectSearchResultDialog.ui )
    qt4_wrap_cpp( MOC_SOURCES MainDialog.h ObjectPlacePropertyDialog.h ObjectSearchResultDialog.h )
    include_directories( ${CMAKE_BINARY_DIR}/subarchitectures/conceptual.sa/src/c++/conceptual/components/Tester )

    add_cast_component(ConceptualTester Tester.cc MainDialog.cc ObjectPlacePropertyDialog.cc ObjectSearchResultDialog.cc ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS})
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData QtGui)

endif(BUILD_SA_CONCEPTUAL_TESTER)

