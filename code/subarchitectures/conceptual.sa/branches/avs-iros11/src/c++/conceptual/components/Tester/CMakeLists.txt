option(BUILD_SA_CONCEPTUAL_TESTER "Build Conceptual.SA Tester component" YES)
if(BUILD_SA_CONCEPTUAL_TESTER)

    qt4_add_resources( RC_SOURCES resources.qrc )
    qt4_wrap_ui( UI_HEADERS MainDialog.ui ConceptualWidget/ConceptualWidget.ui ConceptualWidget/ObjectPlacePropertyDialog.ui 
                            ConceptualWidget/ObjectSearchResultDialog.ui ConceptualWidget/GraphDialog.ui 
                            ConceptualWidget/RCVisualizer.ui AVSMainWidget/AVSMainWidget.ui ConceptualWidget/AddGroundtruthDialog.ui)
    qt4_wrap_cpp( MOC_SOURCES MainDialog.h ConceptualWidget/ConceptualWidget.h ConceptualWidget/ObjectPlacePropertyDialog.h 
                              ConceptualWidget/ObjectSearchResultDialog.h ConceptualWidget/GraphDialog.h 
                              ConceptualWidget/RCVisualizer.h ConceptualWidget/AddGroundtruthDialog.h
                              AVSMainWidget/AVSMainWidget.h)
    include_directories( ${CMAKE_BINARY_DIR}/subarchitectures/conceptual.sa/src/c++/conceptual/components/Tester )
    include_directories(.)
    include_directories(./ConceptualWidget)
    include_directories(./AVSMainWidget)
    include_directories(../../../../../../vision.sa/src/c++/vision/autogen)
    include_directories(../../../../../../../tools/hardware/video/src/c++/autogen/)

    add_cast_component(ConceptualTester Tester.cc MainDialog.cc ConceptualWidget/ConceptualWidget.cc ConceptualWidget/ObjectPlacePropertyDialog.cc 
                                        ConceptualWidget/ObjectSearchResultDialog.cc ConceptualWidget/GraphDialog.cc ConceptualWidget/AddGroundtruthDialog.cc
                                        AVSMainWidget/AVSMainWidget.cpp ConceptualWidget/RCVisualizer.cc ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS})
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData QtGui ConceptualShared DefaultData SpatialData 
                                               SpatialProperties SpatialProbabilities FrontierInterface VisionData)

endif(BUILD_SA_CONCEPTUAL_TESTER)

