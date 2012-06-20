option(BUILD_SA_CONCEPTUAL_TESTER "Build Conceptual.SA Tester component" YES)
if(BUILD_SA_CONCEPTUAL_TESTER)

    Find_package(Cure REQUIRED)
    include_directories(${CURE_INSTALLROOT}/include/cure)
    link_directories(${CURE_INSTALLROOT}/lib/cure)
    include_directories(${COGX_ROOT}/tools/hardware/utils/src/c++/components/)

    qt4_add_resources( RC_SOURCES resources.qrc )
    qt4_wrap_ui( UI_HEADERS_EVENT_VISUALIZER ConceptualWidget/EventVisualizer.ui ConceptualWidget/AddGroundtruthDialog.ui)
    qt4_wrap_cpp( MOC_SOURCES_EVENT_VISUALIZER ConceptualWidget/EventVisualizer.h ConceptualWidget/AddGroundtruthDialog.h ConceptualWidget/EventView.h)
    qt4_wrap_ui( UI_HEADERS MainDialog.ui ConceptualWidget/ConceptualWidget.ui ConceptualWidget/ObjectPlacePropertyDialog.ui 
                            ConceptualWidget/ObjectSearchResultDialog.ui ConceptualWidget/GraphDialog.ui 
                            ConceptualWidget/EventVisualizer.ui AVSMainWidget/AVSMainWidget.ui 
                            DefaultWidget/DefaultWidget.ui NavWidget/NavWidget.ui DemoWidget/DemoWidget.ui 
                            CategoricalWidget/CategoricalWidget.ui ConceptualWidget/HumanAssertionDialog.ui)
    qt4_wrap_cpp( MOC_SOURCES MainDialog.h ConceptualWidget/ConceptualWidget.h ConceptualWidget/ObjectPlacePropertyDialog.h 
                              ConceptualWidget/ObjectSearchResultDialog.h ConceptualWidget/GraphDialog.h 
                              ConceptualWidget/EventVisualizer.h NavWidget/NavWidget.h 
                              AVSMainWidget/AVSMainWidget.h DefaultWidget/DefaultWidget.h
                              DemoWidget/DemoWidget.h CategoricalWidget/CategoricalWidget.h
                              CategoricalWidget/LaserScanView.h CategoricalWidget/OdometryView.h 
                              ConceptualWidget/HumanAssertionDialog.h)
    include_directories( ${CMAKE_BINARY_DIR}/subarchitectures/conceptual.sa/src/c++/conceptual/components/Tester )
    include_directories(.)
    include_directories(./ConceptualWidget)
    include_directories(./DefaultWidget)
    include_directories(./NavWidget)
    include_directories(./DemoWidget)
    include_directories(./CategoricalWidget)
    include_directories(./AVSMainWidget)
    include_directories(../../autogen)
    include_directories(../../../../../../vision.sa/src/c++/vision/autogen)
    include_directories(../../../../../../../tools/hardware/video/src/c++/autogen/)
    include_directories(../../../../../../spatial.sa/src/c++/)
    include_directories(${COGX_ROOT}/tools/math/src/c++/math/)
    include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision)
    

    add_cast_component(ConceptualTester Tester.cc MainDialog.cc ConceptualWidget/ConceptualWidget.cc ConceptualWidget/ObjectPlacePropertyDialog.cc 
                                        ConceptualWidget/ObjectSearchResultDialog.cc ConceptualWidget/GraphDialog.cc ConceptualWidget/AddGroundtruthDialog.cc
                                        AVSMainWidget/AVSMainWidget.cpp ConceptualWidget/EventVisualizer.cc ConceptualWidget/ConceptualEvent.cc
                                        NavWidget/NavWidget.cc DemoWidget/DemoWidget.cc CategoricalWidget/CategoricalWidget.cc
                                        DefaultWidget/DefaultWidget.cc ConceptualWidget/EventView.cc
                                        CategoricalWidget/LaserScanView.cc CategoricalWidget/OdometryView.cc
                                        ConceptualWidget/HumanAssertionDialog.cc
                                        ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} ${MOC_SOURCES_EVENT_VISUALIZER} ${UI_HEADERS_EVENT_VISUALIZER} )
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData QtGui ConceptualShared DefaultData SpatialData 
                                               SpatialProperties SpatialProbabilities FrontierInterface VisionData
                                               CategoricalData)
    link_cast_component(${CAST_COMPONENT_NAME} CureHWUtils CureUtils CureMath CureTransformation CureGeometry CureSensorData 
                                               CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)

    add_executable(event-visualizer event-visualizer.cc ConceptualWidget/EventVisualizer.cc ConceptualWidget/ConceptualEvent.cc
                                    ConceptualWidget/AddGroundtruthDialog.cc ConceptualWidget/EventView.cc
                                    ${MOC_SOURCES_EVENT_VISUALIZER} ${UI_HEADERS_EVENT_VISUALIZER} ${RC_SOURCES})
    link_cast_component(event-visualizer ConceptualData  ${QT_LIBRARIES} ${QT_QTSVG_LIBRARY})
    install(TARGETS event-visualizer RUNTIME DESTINATION bin ARCHIVE DESTINATION bin)


endif(BUILD_SA_CONCEPTUAL_TESTER)

