option(BUILD_SA_CONCEPTUAL_TESTER "Build Conceptual.SA Tester component" YES)
if(BUILD_SA_CONCEPTUAL_TESTER)

    qt4_add_resources( RC_SOURCES resources.qrc )
    qt4_wrap_ui( UI_HEADERS ConceptualWidget/MainDialog.ui ConceptualWidget/ObjectPlacePropertyDialog.ui ConceptualWidget/ObjectSearchResultDialog.ui ConceptualWidget/GraphDialog.ui )
    qt4_wrap_cpp( MOC_SOURCES ConceptualWidget/MainDialog.h ConceptualWidget/ObjectPlacePropertyDialog.h ConceptualWidget/ObjectSearchResultDialog.h ConceptualWidget/GraphDialog.h )
    include_directories( ${CMAKE_BINARY_DIR}/subarchitectures/conceptual.sa/src/c++/conceptual/components/Tester )
    include_directories(.)
    include_directories(./ConceptualWidget)

    add_cast_component(ConceptualTester Tester.cc ConceptualWidget/MainDialog.cc ConceptualWidget/ObjectPlacePropertyDialog.cc 
						  ConceptualWidget/ObjectSearchResultDialog.cc ConceptualWidget/GraphDialog.cc ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS})
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData QtGui ConceptualShared DefaultData SpatialData SpatialProperties SpatialProbabilities)

endif(BUILD_SA_CONCEPTUAL_TESTER)

