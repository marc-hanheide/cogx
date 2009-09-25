project (VISIONSA)

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})


include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/stereo/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/stereo/src/c++/autogen)


# auto generated code
include_directories(src/c++/vision/autogen)

add_subdirectory(src/c++/vision/autogen)

#add_subdirectory (src/c++/vision/components/VisualWMVisualisation)
option(BUILD_SA_VISION_FERNS "build FERNS detector" YES)
if (BUILD_SA_VISION_FERNS)
add_subdirectory (src/c++/vision/components/ObjectDetectorFERNS)
endif (BUILD_SA_VISION_FERNS)
#add_subdirectory (src/c++/vision/components/DummyDriver)
add_subdirectory (src/c++/vision/components/VideoViewer)
option(BUILD_SA_VISION_STEREOVIEWER "build stereo viewer (needs hardware/stereo)" YES)
if(BUILD_SA_VISION_STEREOVIEWER)
add_subdirectory (src/c++/vision/components/StereoViewer)
endif(BUILD_SA_VISION_STEREOVIEWER)

option(BUILD_SA_VISION_OBJTRACKER "build object tracker" NO)
if (BUILD_SA_VISION_OBJTRACKER)
add_subdirectory (src/c++/vision/components/ObjectTracker)
add_subdirectory (src/c++/vision/components/ObjectTracker2)
add_subdirectory (src/c++/vision/components/ObjectTrackerDriver)
add_subdirectory (src/c++/vision/components/ObjectTrackerTest)
endif (BUILD_SA_VISION_OBJTRACKER)

add_subdirectory (src/c++/vision/components/VisionDriver)
option(BUILD_SA_VISION_RECONSTRUCTION "build visual reconstruction" NO)
if (BUILD_SA_VISION_RECONSTRUCTION)
add_subdirectory (src/c++/vision/components/Reconstruction)
endif (BUILD_SA_VISION_RECONSTRUCTION)

option(BUILD_SA_VISION_OBJDETECTOR "build object detector" NO)
if (BUILD_SA_VISION_OBJDETECTOR)
add_subdirectory (src/c++/vision/components/ObjectDetector)
add_subdirectory (src/c++/vision/components/ObjectDetectorDriver)
endif (BUILD_SA_VISION_OBJDETECTOR)

option(BUILD_SA_VISION_PLANEPOPOUT "build plane pop-out (needs hardware/stereo)" YES)
if(BUILD_SA_VISION_PLANEPOPOUT)
add_subdirectory (src/c++/vision/components/PlanePopOut)
endif(BUILD_SA_VISION_PLANEPOPOUT)

option(BUILD_SA_VISION_SOIFILTER "build SOI filter" YES)
if(BUILD_SA_VISION_SOIFILTER)
add_subdirectory (src/c++/vision/components/SOIFilter)
endif(BUILD_SA_VISION_SOIFILTER)

option(BUILD_SA_VISION_OBJRECOGNIZER "build object recognizer" NO)
if (BUILD_SA_VISION_OBJRECOGNIZER)
add_subdirectory (src/c++/vision/components/ObjectRecognizer)
add_subdirectory (src/python/ObjectRecognizer)
endif (BUILD_SA_VISION_OBJRECOGNIZER)

option(BUILD_VISUAL_LEARNER "Build UOL visual learning component" NO) # default NO until fully integrated
option(BUILD_VISUAL_LEARNER_CTF "Build Matlab CTF for UOL visual learner; off=use prebuilt CTF from SVN" NO)
mark_as_advanced(BUILD_VISUAL_LEARNER_CTF)
if (BUILD_VISUAL_LEARNER)
   if (BUILD_VISUAL_LEARNER_CTF)
      add_subdirectory(src/matlab)
   else (BUILD_VISUAL_LEARNER_CTF)
      add_subdirectory(prebuilt/VisualLearner)   
   endif (BUILD_VISUAL_LEARNER_CTF)


   find_file( # TODO: Why is FIND_LIBRARY (FeatureLearningProxy) not working????
      VISUAL_LEARNER_MATLAB_PROXY
       NAMES libVisualLearnerProxy.so
      PATHS ${OUTPUT}  ${OUTPUT}/lib
      NO_DEFAULT_PATH
      DOC "Matlab Library required for VisualLearner"
   )
   if (VISUAL_LEARNER_MATLAB_PROXY)
      add_subdirectory(src/c++/vision/components/VisualLearner)
   else(VISUAL_LEARNER_MATLAB_PROXY)
      message(
         "VisualLearnerProxy library not installed, yet.\n"
         "   Run 'make install' to generate and install it.\n"
         "   Then run 'ccmake' again.\n"
         "   (For now, VisualLearner won't be built)\n"
         "   (see also: BUILD_VISUAL_LEARNER_CTF)"
      )
      endif (VISUAL_LEARNER_MATLAB_PROXY)
endif (BUILD_VISUAL_LEARNER)
