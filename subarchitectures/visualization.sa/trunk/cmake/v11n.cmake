# Include the visualization subarchitecture
# To be included in the top-level CogX CMakeLists.txt

macro(relative_path OUTPUT BASE TARGET)
   string(REGEX REPLACE "^${BASE}/" "" _temp ${TARGET})
   set(${OUTPUT} "${_temp}")
endmacro(relative_path)

option(BUILD_SA_V11N "Build visualization libraries (Requires QT4)" NO)
if(BUILD_SA_V11N)
   get_filename_component(VISUALIZATION_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/.. ABSOLUTE)
   relative_path(VISUALIZATION_ROOT_DIR_REL ${COGX_ROOT} ${VISUALIZATION_ROOT_DIR})

   set(VISUALIZATION_INCLUDE_DIRS
      ${VISUALIZATION_ROOT_DIR}/src/c++/autogen
      ${VISUALIZATION_ROOT_DIR}/src/c++/core/client
      )
   set(VISUALIZATION_LIBRARIES
      DisplayClient VisualizationData
      )
   add_subdirectory(${VISUALIZATION_ROOT_DIR_REL})
   add_definitions(-DFEAT_VISUALIZATION)
endif(BUILD_SA_V11N)

