include_directories(.)

# Qt4 related processing
set(MOC_HEADERS MonitorDialog.h PreviewDialog.h SaveDataDialog.h MapDialog.h WMDialog.h StatsDialog.h ResultsDialog.h NodesDialog.h PullNodePlaceInfoDialog.h)
set(RESOURCES Monitor.qrc)
set(UIS MonitorDialog.ui PreviewDialog.ui SaveDataDialog.ui MapDialog.ui WMDialog.ui StatsDialog.ui ResultsDialog.ui NodesDialog.ui PullNodePlaceInfoDialog.ui)

qt4_add_resources( RC_SOURCES ${RESOURCES} )
qt4_wrap_ui( UI_HEADERS ${UIS} )
qt4_wrap_cpp( MOC_SOURCES ${MOC_HEADERS} )
include_directories( ${CMAKE_BINARY_DIR}/subarchitectures/place.sa/src/c++/components/Monitor )

# Component
set(SOURCES Monitor.cpp MonitorDialog.cpp PreviewDialog.cpp SaveDataDialog.cpp MapDialog.cpp WMDialog.cpp StatsDialog.cpp ResultsDialog.cpp NodesDialog.cpp PullNodePlaceInfoDialog.cpp)
add_cast_component(PlaceMonitor ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS})
link_cast_component(${CAST_COMPONENT_NAME} PlaceData PlaceShared)
link_cast_component(${CAST_COMPONENT_NAME} crfh)
link_cast_component(${CAST_COMPONENT_NAME} ${QT_LIBRARIES})
