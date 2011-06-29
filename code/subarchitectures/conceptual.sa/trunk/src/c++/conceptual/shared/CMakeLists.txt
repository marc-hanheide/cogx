project(ConceptualShared)

set(SOURCES VariableNameGenerator.cc SvgView.cc GraphPlotter.cc)
#qt4_add_resources( RC_SOURCES resources.qrc )
#qt4_wrap_ui( UI_HEADERS  )
qt4_wrap_cpp( MOC_SOURCES SvgView.h)
include_directories( ${CMAKE_BINARY_DIR}/subarchitectures/conceptual.sa/src/c++/conceptual/shared )


add_library(${PROJECT_NAME} SHARED ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS})
target_link_libraries(${PROJECT_NAME} CDL ConceptualData QtGui QtCore QtSvg)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
