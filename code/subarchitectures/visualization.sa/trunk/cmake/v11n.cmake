# Include the visualization subarchitecture
# To be included in the top-level CogX CMakeLists.txt

option(BUILD_SA_V11N "Build visualization libraries (Requires QT4)" NO)
set(v11n_subdir subarchitectures/visualization.sa)

# Conditionally included subdirectory
cogx_add_subdir(BUILD_SA_V11N ${v11n_subdir})

if(BUILD_SA_V11N)
   set(VISUALIZATION_INCLUDE_DIRS
      ${COGX_ROOT}/${v11n_subdir}/src/c++/autogen
      ${COGX_ROOT}/${v11n_subdir}/src/c++/core/client
      )
   set(VISUALIZATION_LIBRARIES
      DisplayClient VisualizationData
      )
   add_definitions(-DFEAT_VISUALIZATION)
   add_definitions(-DHAVE_COGX_MATH)
   add_definitions(-DHAVE_COGX_VIDEO)
endif(BUILD_SA_V11N)

