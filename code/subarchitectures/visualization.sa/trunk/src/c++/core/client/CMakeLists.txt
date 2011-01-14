
project(VisualizationClientCore)

#set(VERBOSE 1)

option(BUILD_SA_V11N_CLIENT "Build the visualisation display client library" YES)
if(BUILD_SA_V11N_CLIENT)

   find_package(OpenCV REQUIRED)
   include_directories(${OPENCV_INCLUDE_DIRS})
   link_directories(${OPENCV_LIBRARY_DIRS})
   add_definitions(-DFEAT_VISUALIZATION_OPENCV)

   add_cast_component(DisplayClient CDisplayClient.cpp CFormValues.cpp)
   link_cast_component(${CAST_COMPONENT_NAME}
      VisualizationData Math Video VideoUtils
      # CogxIceAutogen
      ${OPENCV_LDFLAGS}
      )

endif(BUILD_SA_V11N_CLIENT)


