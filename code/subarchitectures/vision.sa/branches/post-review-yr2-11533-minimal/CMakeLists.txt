project (VISIONSA)

# use cmake files packaged with CAST
include(UseCAST)

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
add_subdirectory(src/c++/vision/libs)

SET(VISION_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/c++/vision/libs)

#add_subdirectory (src/c++/vision/components/VisualWMVisualisation)
option(BUILD_SA_VISION_FERNS "build FERNS detector" NO)
if (BUILD_SA_VISION_FERNS)
add_subdirectory (src/c++/vision/components/ObjectDetectorFERNS)
endif (BUILD_SA_VISION_FERNS)

option(BUILD_SA_VISION_UTIL_COMPS "build dummy driver, video viewer, video grabber" YES)
if (BUILD_SA_VISION_UTIL_COMPS)
add_subdirectory (src/c++/vision/components/DummyDriver)
add_subdirectory (src/c++/vision/components/VideoViewer)
add_subdirectory (src/c++/vision/components/VideoGrabber)
endif (BUILD_SA_VISION_UTIL_COMPS)

option(BUILD_SA_VISION_STEREOVIEWER "build stereo viewer (needs hardware/stereo)" NO)
if(BUILD_SA_VISION_STEREOVIEWER)
add_subdirectory (src/c++/vision/components/StereoViewer)
endif(BUILD_SA_VISION_STEREOVIEWER)

option(BUILD_SA_VISION_PEOPLE_DETECTOR "build people detector" NO)
if (BUILD_SA_VISION_PEOPLE_DETECTOR)
add_subdirectory (src/c++/vision/components/PeopleDetector)
add_subdirectory (src/c++/vision/components/PeopleDetectorTest)
endif (BUILD_SA_VISION_PEOPLE_DETECTOR)


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
add_subdirectory (src/c++/vision/components/StereoDetectorReasoner)
add_subdirectory (src/c++/vision/components/StereoDetectorDriver)
endif (BUILD_SA_VISION_STEREODETECTOR)

option(BUILD_SA_VISION_PLANEPOPOUT "build plane pop-out (needs hardware/stereo)" NO)
if(BUILD_SA_VISION_PLANEPOPOUT)
add_subdirectory (src/c++/vision/components/PlanePopOut)
endif(BUILD_SA_VISION_PLANEPOPOUT)

option(BUILD_SA_VISION_SOIFILTER "build SOI filter" NO)
if(BUILD_SA_VISION_SOIFILTER)
add_subdirectory (src/c++/vision/components/SOIFilter)
endif(BUILD_SA_VISION_SOIFILTER)

option(BUILD_SA_VISION_OBJANALYZER "build Object Analyzer" NO)
if(BUILD_SA_VISION_OBJANALYZER)
add_subdirectory (src/c++/vision/components/ObjectAnalyzer)
endif(BUILD_SA_VISION_OBJANALYZER)

option(BUILD_SA_VISION_VISMEDIATOR "build Visual Mediator" NO)
if(BUILD_SA_VISION_VISMEDIATOR)
add_subdirectory (src/c++/vision/components/VisualMediator)
endif(BUILD_SA_VISION_VISMEDIATOR)

option(BUILD_SA_VISION_VIRTUALSCENE "build 3D Virtual Scene" NO)
if(BUILD_SA_VISION_VIRTUALSCENE)
add_subdirectory (src/c++/vision/components/VirtualScene)
endif(BUILD_SA_VISION_VIRTUALSCENE)

option(BUILD_SA_VISION_VIRTUALSCENE_TEST "build 3D Virtual Scene Test" NO)
if(BUILD_SA_VISION_VIRTUALSCENE_TEST)
add_subdirectory (src/c++/vision/components/VirtualSceneTest)
endif(BUILD_SA_VISION_VIRTUALSCENE_TEST)

option(BUILD_SA_VISION_VIRTUALSCENE_2D "build 2D Virtual Scene" NO)
if(BUILD_SA_VISION_VIRTUALSCENE_2D)
add_subdirectory (src/c++/vision/components/VirtualScene2D)
endif(BUILD_SA_VISION_VIRTUALSCENE_2D)

option(BUILD_SA_VISION_OBJRECOGNIZER "build object recognizer" NO)
if (BUILD_SA_VISION_OBJRECOGNIZER)
   add_subdirectory (src/c++/vision/components/ObjectRecognizer)
endif (BUILD_SA_VISION_OBJRECOGNIZER)

option(BUILD_SA_VISION_OBJRECOGNIZER_3D "build Object Recognizer" NO)
if (BUILD_SA_VISION_OBJRECOGNIZER_3D)
add_subdirectory (src/c++/vision/components/ObjectRecognizer3D)
add_subdirectory (src/c++/vision/components/ObjectRecognizer3DDriver)
endif (BUILD_SA_VISION_OBJRECOGNIZER_3D)

option(BUILD_SA_VISION_OBJRECOGNIZER_3D2 "build Object Recognizer2 (for George)" NO)
if (BUILD_SA_VISION_OBJRECOGNIZER_3D2)
add_subdirectory (src/c++/vision/components/ObjectRecognizer3D2)
endif (BUILD_SA_VISION_OBJRECOGNIZER_3D2)

option(BUILD_SA_VISION_DISPLAY_CONVEXHULL_PB "build convex hull display" NO)
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

option(BUILD_SA_VISION_SIMULATION "Build Simulated Vision" NO)
if(BUILD_SA_VISION_SIMULATION)
   add_subdirectory (src/c++/vision/components/SimulatedVision)
endif(BUILD_SA_VISION_SIMULATION)

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

option(BUILD_SA_VISION_PG_VIEWER "build PointGrey Viewer" NO)
if(BUILD_SA_VISION_PG_VIEWER)
add_subdirectory (src/c++/vision/components/PointGreyViewer)
endif(BUILD_SA_VISION_PG_VIEWER)

option(BUILD_SA_VISION_TEST_COMP "build test component" NO)
if(BUILD_SA_VISION_TEST_COMP)
add_subdirectory (src/c++/vision/components/TestComponent)
add_subdirectory (src/c++/vision/components/TestComponentReceiver)
endif(BUILD_SA_VISION_TEST_COMP)

option(BUILD_SA_VISION_SOI2ROI "build SOI2ROI" NO)
if(BUILD_SA_VISION_SOI2ROI)
add_subdirectory (src/c++/vision/components/SOI2ROI)
endif(BUILD_SA_VISION_SOI2ROI)

option(BUILD_SA_VISION_SHAPEDESCRIPTOR_3D "build 3D shape descriptor" NO)
if(BUILD_SA_VISION_SHAPEDESCRIPTOR_3D)
add_subdirectory (src/c++/vision/components/ShapeDescriptor3D)
endif(BUILD_SA_VISION_SHAPEDESCRIPTOR_3D)

if(BUILD_PYTHON_COMPONENTS)
   add_subdirectory (src/python)
endif(BUILD_PYTHON_COMPONENTS)

