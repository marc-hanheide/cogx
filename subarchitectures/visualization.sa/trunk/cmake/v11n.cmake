# Include the visualization subarchitecture
# To be included in the top-level CogX CMakeLists.txt

option(BUILD_SA_V11N "Build visualization libraries (Requires QT4)" NO)
set(v11n_subdir subarchitectures/visualization.sa)

# Conditionally included subdirectory
cogx_add_subdir(BUILD_SA_V11N ${v11n_subdir})

set(VISUALIZATION_INCLUDE_DIRS
   ${COGX_ROOT}/${v11n_subdir}/src/c++/autogen
   ${COGX_ROOT}/${v11n_subdir}/src/c++/core/client
   )
set(VISUALIZATION_LIBRARIES
   DisplayClient VisualizationData
   )
add_definitions(-DFEAT_VISUALIZATION)

if (1)
  add_definitions(-DHAVE_COGX_MATH)
  set(VISUALIZATION_INCLUDE_DIRS ${VISUALIZATION_INCLUDE_DIRS}
    ${COGX_ROOT}/tools/math/src/c++/autogen
    )
  set(VISUALIZATION_LIBRARIES ${VISUALIZATION_LIBRARIES}
    # Math
    )
endif()

if (1)
  add_definitions(-DHAVE_COGX_VIDEO)
  set(VISUALIZATION_INCLUDE_DIRS ${VISUALIZATION_INCLUDE_DIRS}
    ${COGX_ROOT}/tools/hardware/video/src/c++/autogen
    )
  set(VISUALIZATION_LIBRARIES ${VISUALIZATION_LIBRARIES}
    # Video
    )
endif()

if(NOT BUILD_SA_V11N)
   set(VISUALIZATION_LIBRARY_DIRS ${COGX_ROOT}/output/lib)
endif()

