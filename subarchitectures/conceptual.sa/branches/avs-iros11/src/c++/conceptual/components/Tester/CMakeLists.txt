option(BUILD_SA_CONCEPTUAL_TESTER "Build Conceptual.SA Tester component" YES)
if(BUILD_SA_CONCEPTUAL_TESTER)

    qt4_add_resources( RC_SOURCES resources.qrc )
    qt4_wrap_ui( UI_HEADERS MainDialog.ui ConceptualWidget/ConceptualWidget.ui ConceptualWidget/ObjectPlacePropertyDialog.ui 
                            ConceptualWidget/ObjectSearchResultDialog.ui ConceptualWidget/GraphDialog.ui 
                            AVSMainWidget/AVSMainWidget.ui)
    qt4_wrap_cpp( MOC_SOURCES MainDialog.h ConceptualWidget/ConceptualWidget.h ConceptualWidget/ObjectPlacePropertyDialog.h 
                              ConceptualWidget/ObjectSearchResultDialog.h ConceptualWidget/GraphDialog.h 
                              AVSMainWidget/AVSMainWidget.h)
    include_directories( ${CMAKE_BINARY_DIR}/subarchitectures/conceptual.sa/src/c++/conceptual/components/Tester )
    include_directories(.)
    include_directories(./ConceptualWidget)

    add_cast_component(ConceptualTester Tester.cc MainDialog.cc ConceptualWidget/ConceptualWidget.cc ConceptualWidget/ObjectPlacePropertyDialog.cc 
                                        ConceptualWidget/ObjectSearchResultDialog.cc ConceptualWidget/GraphDialog.cc 
                                        AVSMainWidget/AVSMainWidget.cpp ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS})
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData QtGui ConceptualShared DefaultData SpatialData SpatialProperties SpatialProbabilities)

endif(BUILD_SA_CONCEPTUAL_TESTER)

