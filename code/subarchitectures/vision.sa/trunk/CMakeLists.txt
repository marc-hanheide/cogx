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
add_subdirectory (src/c++/vision/components/DummyDriver)
add_subdirectory (src/c++/vision/components/VideoViewer)
option(BUILD_SA_VISION_STEREOVIEWER "build stereo viewer (needs hardware/stereo)" YES)
if(BUILD_SA_VISION_STEREOVIEWER)
add_subdirectory (src/c++/vision/components/StereoViewer)
endif(BUILD_SA_VISION_STEREOVIEWER)

option(BUILD_SA_VISION_OBJTRACKER "build object tracker" NO)
if (BUILD_SA_VISION_OBJTRACKER)
add_subdirectory (src/c++/vision/components/ObjectTracker)
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

option(BUILD_SA_VISION_STEREOFLAPDETECTOR "build stereo flap detector" NO)
if (BUILD_SA_VISION_STEREOFLAPDETECTOR)
add_subdirectory (src/c++/vision/components/StereoFlapDetector)
add_subdirectory (src/c++/vision/components/StereoFlapDetectorDriver)
endif (BUILD_SA_VISION_STEREOFLAPDETECTOR)

option(BUILD_SA_VISION_STEREODETECTOR "build stereo detector" NO)
if (BUILD_SA_VISION_STEREODETECTOR)
add_subdirectory (src/c++/vision/components/StereoDetector)
add_subdirectory (src/c++/vision/components/StereoDetectorDriver)
endif (BUILD_SA_VISION_STEREODETECTOR)

option(BUILD_SA_VISION_PLANEPOPOUT "build plane pop-out (needs hardware/stereo)" YES)
if(BUILD_SA_VISION_PLANEPOPOUT)
add_subdirectory (src/c++/vision/components/PlanePopOut)
endif(BUILD_SA_VISION_PLANEPOPOUT)

option(BUILD_SA_VISION_SOIFILTER "build SOI filter" YES)
if(BUILD_SA_VISION_SOIFILTER)
add_subdirectory (src/c++/vision/components/SOIFilter)
endif(BUILD_SA_VISION_SOIFILTER)

option(BUILD_SA_VISION_OBJANALYZER "build Object Analyzer" YES)
if(BUILD_SA_VISION_OBJANALYZER)
add_subdirectory (src/c++/vision/components/ObjectAnalyzer)
endif(BUILD_SA_VISION_OBJANALYZER)

option(BUILD_SA_VISION_VISMEDIATOR "build Visual Mediator" YES)
if(BUILD_SA_VISION_VISMEDIATOR)
add_subdirectory (src/c++/vision/components/VisualMediator)
endif(BUILD_SA_VISION_VISMEDIATOR)

option(BUILD_SA_VISION_VIRTUALSCENE "build Virtual Scene" YES)
if(BUILD_SA_VISION_VIRTUALSCENE)
add_subdirectory (src/c++/vision/components/VirtualScene)
endif(BUILD_SA_VISION_VIRTUALSCENE)

option(BUILD_SA_VISION_OBJRECOGNIZER "build object recognizer" NO)
if (BUILD_SA_VISION_OBJRECOGNIZER)
add_subdirectory (src/c++/vision/components/ObjectRecognizer)
add_subdirectory (src/python/ObjectRecognizer)
endif (BUILD_SA_VISION_OBJRECOGNIZER)

option(BUILD_SA_VISION_OBJRECOGNIZER_3D "build Object Recognizer" NO)
if (BUILD_SA_VISION_OBJRECOGNIZER_3D)
add_subdirectory (src/c++/vision/components/ObjectRecognizer3D)
add_subdirectory (src/c++/vision/components/Driver/ObjectRecognizer3D)
endif (BUILD_SA_VISION_OBJRECOGNIZER_3D)

option(BUILD_SA_VISION_DISPLAY_CONVEXHULL_PB "build convex hull display" YES)
if (BUILD_SA_VISION_DISPLAY_CONVEXHULL_PB)
add_subdirectory (src/c++/vision/components/DisplayConvexHullPB)
endif (BUILD_SA_VISION_DISPLAY_CONVEXHULL_PB)

option(BUILD_SA_VISION_VISLEARNER "Build UOL visual learning component" NO) # default NO until fully integrated
option(BUILD_SA_VISION_VISLEARNER_CTF "Build Matlab CTF for UOL visual learner; off=use prebuilt CTF" NO)
mark_as_advanced(BUILD_SA_VISION_VISLEARNER_CTF)

if (BUILD_SA_VISION_VISLEARNER_CTF)
   add_subdirectory(src/matlab)
elseif (BUILD_SA_VISION_VISLEARNER)
   add_subdirectory(prebuilt/VisualLearner)   
endif (BUILD_SA_VISION_VISLEARNER_CTF)

if (BUILD_SA_VISION_VISLEARNER)
   set (LIB_VLPROXY_SO "libVisualLearnerProxy.so")
   file(GLOB VISUAL_LEARNER_MATLAB_PROXY ${OUTPUT}/lib/${LIB_VLPROXY_SO})
   if (VISUAL_LEARNER_MATLAB_PROXY)
      add_subdirectory(src/c++/vision/components/VisualLearner)
   else(VISUAL_LEARNER_MATLAB_PROXY)
      message(
         "${LIB_VLPROXY_SO} not installed, yet.\n"
         "   Run 'make install' to generate and install it.\n"
         "   Then run 'ccmake' again.\n"
         "   (For now, VisualLearner won't be built)\n"
         "   (see also: BUILD_SA_VISION_VISLEARNER_CTF - Advanced)"
      )
   endif (VISUAL_LEARNER_MATLAB_PROXY)
endif (BUILD_SA_VISION_VISLEARNER)

option(BUILD_SA_VISION_CAMERA_MOUNT "build camera mount" YES)
if (BUILD_SA_VISION_CAMERA_MOUNT)
add_subdirectory (src/c++/vision/components/CameraMount)
endif (BUILD_SA_VISION_CAMERA_MOUNT)

