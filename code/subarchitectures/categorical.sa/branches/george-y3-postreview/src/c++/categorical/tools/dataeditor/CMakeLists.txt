project(dataeditor)

# Set the sources.
set(SOURCES main.cpp MainWindow.cpp ProgressDialog.cpp LabelDialog.cpp)
set(MOC_HEADERS MainWindow.h ProgressDialog.h LabelDialog.h)
set(RESOURCES resources/resources.qrc)
set(UIS MainWindow.ui ProgressDialog.ui LabelDialog.ui)

qt4_add_resources( RC_SOURCES ${RESOURCES} )
qt4_wrap_ui( UI_HEADERS ${UIS} )
qt4_wrap_cpp( MOC_SOURCES ${MOC_HEADERS} )
include_directories( ${CMAKE_BINARY_DIR}/subarchitectures/categorical.sa/src/c++/categorical/tools/dataeditor )

add_executable(${PROJECT_NAME} ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
target_link_libraries(${PROJECT_NAME} CategoricalData CategoricalShared)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin ARCHIVE DESTINATION bin)
