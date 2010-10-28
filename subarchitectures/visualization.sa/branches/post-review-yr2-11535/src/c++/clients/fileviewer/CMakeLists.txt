


project(FileMonitorDisplayClient)

option(BUILD_SA_V11N_CLIENT_FILE_MONITOR "Build file monitor visualisation client" ON)

if(BUILD_SA_V11N_CLIENT_FILE_MONITOR)

find_package(Boost 1.36.0 REQUIRED COMPONENTS regex )

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
   ${Boost_LIBRARIES}
#   boost_regex-mt
   )

endif(BUILD_SA_V11N_CLIENT_FILE_MONITOR)