# Include the visualization subarchitecture
# To be included in the top-level CogX CMakeLists.txt

option(BUILD_SA_V11N "Build visualization libraries (Requires QT4)" NO)
if(BUILD_SA_V11N)
   set(VISUALIZATION_INCLUDE_DIRS
      ${COGX_ROOT}/subarchitectures/visualization.sa/src/c++/autogen
      ${COGX_ROOT}/subarchitectures/visualization.sa/src/c++/core/client
      )
   set(VISUALIZATION_LIBRARIES
      DisplayClient VisualizationData
      )
   add_subdirectory(subarchitectures/visualization.sa)
   add_definitions(-DFEAT_VISUALIZATION)
endif(BUILD_SA_V11N)

