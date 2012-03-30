# Include the visualization subarchitecture
# To be included in the top-level CogX CMakeLists.txt

option(BUILD_SA_V11N "Build visualization libraries (Requires QT4)" NO)
set(v11n_subdir subarchitectures/visualization.sa)

# Conditionally included subdirectory
cogx_add_subdir(BUILD_SA_V11N ${v11n_subdir})

set (__v11n_binary_dir ${CMAKE_CURRENT_BINARY_DIR})
set (__v11n_source_dir ${CMAKE_CURRENT_SOURCE_DIR})
set (__v11n_list_file ${CMAKE_CURRENT_LIST_FILE})
#message(" *** ${__v11n_binary_dir}")
#message(" *** ${__v11n_source_dir}")
#message(" *** ${__v11n_list_file}")
string(REPLACE "${__v11n_source_dir}" "" __v11n_rel_list_file "${__v11n_list_file}")
#message(" *** ${__v11n_rel_list_file}")
string(REPLACE "/cmake/v11n.cmake" "" __v11n_sa_dir "${__v11n_rel_list_file}")
#message(" *** ${__v11n_sa_dir}")
set (__v11n_client_binary_dir "${__v11n_binary_dir}${__v11n_sa_dir}/scr/c++/core")
#message(" *** ${__v11n_client_binary_dir}")

# params: ON/OFF [NOMATH NOVIDEO OPENCV]
macro(ENABLE_COGX_VISUALIZATION onoff)
   set (onoff ${onoff})
   if(NOT onoff)
      message("***DISABLED*** _COGX_VISUALIZATION")
      remove_definitions(-DFEAT_VISUALIZATION  -DFEAT_VISUALIZATION_OPENCV)
      set(VISUALIZATION_INCLUDE_DIRS "")
      set(VISUALIZATION_LIBRARY_DIRS "")
      set(VISUALIZATION_LIBRARIES "")
   else()
      message("ENABLE_COGX_VISUALIZATION")
      set (__havemath 1)
      set (__havevideo 1)
      set (__haveopencv 0)
      foreach(arg ${ARGN})
         if(arg STREQUAL "NOMATH")
            set (__havemath 0)
            message("NOMATH")
         elseif(arg STREQUAL "NOVIDEO")
            set (__havevideo 0)
            message("NOVIDEO")
         elseif(arg STREQUAL "OPENCV")
            set (__haveopencv 1)
            message("OPENCV")
         endif()
      endforeach()
      set(VISUALIZATION_INCLUDE_DIRS
         ${COGX_ROOT}/${v11n_subdir}/src/c++/autogen
         ${COGX_ROOT}/${v11n_subdir}/src/c++/core/client
         )
      set(VISUALIZATION_LIBRARIES
         DisplayClient VisualizationData
         )
      add_definitions(-DFEAT_VISUALIZATION)
      if (__haveopencv)
         add_definitions(-DFEAT_VISUALIZATION_OPENCV)
      endif()

      if (__havemath)
        add_definitions(-DHAVE_COGX_MATH)
        set(VISUALIZATION_INCLUDE_DIRS ${VISUALIZATION_INCLUDE_DIRS}
          ${COGX_ROOT}/tools/math/src/c++/autogen
          )
        set(VISUALIZATION_LIBRARIES ${VISUALIZATION_LIBRARIES}
          # Math
          )
      endif()

      if (__havevideo)
        add_definitions(-DHAVE_COGX_VIDEO)
        set(VISUALIZATION_INCLUDE_DIRS ${VISUALIZATION_INCLUDE_DIRS}
          ${COGX_ROOT}/tools/hardware/video/src/c++/autogen
          )
        set(VISUALIZATION_LIBRARIES ${VISUALIZATION_LIBRARIES}
          # Video
          )
      endif()

      set(VISUALIZATION_LIBRARY_DIRS ${__v11n_client_binary_dir})
      if(NOT BUILD_SA_V11N)
         set(VISUALIZATION_LIBRARY_DIRS ${VISUALIZATION_LIBRARY_DIRS} ${COGX_ROOT}/output/lib)
      endif()

      include_directories(${VISUALIZATION_INCLUDE_DIRS})
      link_directories(${VISUALIZATION_LIBRARY_DIRS})
   endif()
endmacro()

