
project(FileMonitorDisplayClient)

set( SOURCES
   FileMonitor.cpp
   )

include_directories( .
   ${VISUALIZATION_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIR}
   )
link_directories(${Boost_LIBRARY_DIRS})

add_cast_component(FileMonitorDC ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} 
   # VisionData Video VideoUtils
   DisplayClient VisualizationData
   boost_regex-mt
   )

