
PROJECT(VisualLearnerProxy)
if (NOT BUILD_SA_VISION_VISLEARNER_FAKE)

   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

   # Matlab headers are required to build the proxy
   find_package(MatlabCompiler)
   include_directories( ${MATLAB_INCLUDE_DIR} )

   INCLUDE_DIRECTORIES(${VisualLearnerCtf_BINARY_DIR}/build)
   LINK_DIRECTORIES( ${VisualLearnerCtf_BINARY_DIR}/build )

   SET(SOURCES 
      initlib.cpp  
      Globals.cpp
      MatlabHelper.cpp
      Conversion.cpp
      VisualLearnerProxy.cpp
      #AffordanceLearnerProxy.cpp
      Enumerator.cpp
      LoggerProxy.cpp
      )
   SET(HEADERS initlib.h RecogniserP.h)
   add_library(VisualLearnerProxy SHARED ${SOURCES})
   target_link_libraries(VisualLearnerProxy VisualLearnerCtf)
   target_link_libraries(VisualLearnerProxy VisionData PointCloud VideoUtils)

   find_package(OpenCV REQUIRED)
   include_directories(${OPENCV_INCLUDE_DIRS})
   link_directories(${OPENCV_LIBRARY_DIRS})
   target_link_libraries(VisualLearnerProxy ${OPENCV_LDFLAGS})

   install(TARGETS VisualLearnerProxy LIBRARY DESTINATION lib)
endif (NOT BUILD_SA_VISION_VISLEARNER_FAKE)


if (BUILD_SA_VISION_VISLEARNER_FAKE)

   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

   SET(SOURCES 
      fake/VisualLearnerProxy.cpp
      #fake/AffordanceLearnerProxy.cpp
      Enumerator.cpp
      LoggerProxy.cpp
      )
   add_library(VisualLearnerProxy_fake SHARED ${SOURCES})
   target_link_libraries(VisualLearnerProxy_fake)
   target_link_libraries(VisualLearnerProxy_fake VisionData PointCloud VideoUtils)

   install(TARGETS VisualLearnerProxy_fake LIBRARY DESTINATION lib)
endif (BUILD_SA_VISION_VISLEARNER_FAKE)

