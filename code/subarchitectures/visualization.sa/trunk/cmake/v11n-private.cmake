# Configuration file for clients that are part of the Display Server distribution.

set(VISUALIZATION_INCLUDE_DIRS
   ${CMAKE_CURRENT_SOURCE_DIR}/src/c++/autogen
   ${CMAKE_CURRENT_SOURCE_DIR}/src/c++/core/client
   )
set(VISUALIZATION_LIBRARIES
   DisplayClient VisualizationData
   )
add_definitions(-DFEAT_VISUALIZATION)

