option(BUILD_SA_DEFAULT_TESTER "Build Default.SA Tester component" YES)
if(BUILD_SA_DEFAULT_TESTER)

    # Sources
    set(MOC_HEADERS TesterDialog.h)
    set(RESOURCES Tester.qrc)
    set(UIS TesterDialog.ui)
    set(SOURCES Tester.cc TesterDialog.cc)

    # Qt4 related processing
    find_package( Qt4 REQUIRED )
    include(${QT_USE_FILE})
    qt4_add_resources( RC_SOURCES ${RESOURCES} )
    qt4_wrap_ui( UI_HEADERS ${UIS} )
    qt4_wrap_cpp( MOC_SOURCES ${MOC_HEADERS} )
    include_directories( ${CMAKE_BINARY_DIR}/subarchitectures/default.sa/src/c++/def/components/Tester )

    # CAST Component
    add_cast_component(DefaultTester ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS})
    link_cast_component(${CAST_COMPONENT_NAME} DefaultData)
    link_cast_component(${CAST_COMPONENT_NAME} ${QT_LIBRARIES})

endif(BUILD_SA_DEFAULT_TESTER)

