project (VISIONSA)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/matlab/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)

option(BUILD_VISUAL_LEARNER "Build UOL visual learning component" YES)
option(BUILD_VISUAL_LEARNER_CTF "Build Matlab CTF for UOL visual learner; off=use prebuilt CTF from SVN" NO)
mark_as_advanced(BUILD_VISUAL_LEARNER_CTF)

# auto generated code
include_directories(src/c++/vision/autogen)

add_subdirectory(src/c++/vision/autogen)

#add_subdirectory (src/c++/vision/components/VisualWMVisualisation)
#add_subdirectory (src/c++/vision/components/ObjectDetectorFERNS)
#add_subdirectory (src/c++/vision/components/DummyDriver)
add_subdirectory (src/c++/vision/components/VideoViewer)
add_subdirectory (src/c++/vision/components/ObjectTracker)
add_subdirectory (src/c++/vision/components/ObjectTrackerDriver)

add_subdirectory (src/c++/vision/components/Reconstruction)

add_subdirectory (src/c++/vision/components/ObjectDetector)
add_subdirectory (src/c++/vision/components/ObjectDetectorDriver)

add_subdirectory (src/c++/vision/components/VisualLearner)

IF (BUILD_VISUAL_LEARNER)
   IF (BUILD_VISUAL_LEARNER_CTF)
      add_subdirectory(src/matlab)
   ELSE (BUILD_VISUAL_LEARNER_CTF)
      add_subdirectory(prebuilt/VisualLearner)   
   ENDIF (BUILD_VISUAL_LEARNER_CTF)

   find_file( # TODO: Why is FIND_LIBRARY (FeatureLearningProxy) not working????
      VISUAL_LEARNER_MATLAB_PROXY
      NAMES libVisualLearnerProxy.so
      PATHS ${OUTPUT}  ${OUTPUT}/lib
      NO_DEFAULT_PATH
      DOC "Matlab Library required for VisualLearner"
   )
   IF (VISUAL_LEARNER_MATLAB_PROXY)
      add_subdirectory(src/c++/vision/components/VisualLearner)
   ELSE(VISUAL_LEARNER_MATLAB_PROXY)
      message(
         "VisualLearnerProxy library not installed, yet.\n"
         "   Run 'make install' to generate and install it.\n"
         "   Then run 'ccmake' again.\n"
         "   (For now, VisualLearner won't be built)\n"
         "   (see also: BUILD_VISUAL_LEARNER_CTF)"
      )
      ENDIF (VISUAL_LEARNER_MATLAB_PROXY)
ENDIF (BUILD_VISUAL_LEARNER)
