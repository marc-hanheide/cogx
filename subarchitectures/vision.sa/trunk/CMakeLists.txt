project (VISIONSA)

# use cmake files packaged with CAST
include(UseCAST)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/castutils/src/c++)


# auto generated code
include_directories(src/c++/vision/autogen)
if (DO_SLICE_ONLY)
   return()
endif()

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

add_subdirectory(src/c++/vision/autogen)
add_subdirectory(src/c++/vision/libs)

SET(VISION_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/c++/vision/libs)

#add_subdirectory (src/c++/vision/components/VisualWMVisualisation)
option(BUILD_SA_VISION_FERNS "build FERNS detector" YES)
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

option(BUILD_SA_VISION_PCVIEWER "build point cloud viewer (needs hardware/pointCloud)" YES)
if(BUILD_SA_VISION_PCVIEWER)
add_subdirectory (src/c++/vision/components/PointCloudViewer)
endif(BUILD_SA_VISION_PCVIEWER)

option(BUILD_SA_VISION_PEOPLE_DETECTOR "build people detector" YES)
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

option(BUILD_SA_VISION_TOMGINE "build TomGine libraries and wrapper" YES)
if (BUILD_SA_VISION_TOMGINE)
add_subdirectory (src/c++/vision/components/TomGine)
endif (BUILD_SA_VISION_TOMGINE)

option(BUILD_SA_VISION_STEREODETECTOR "build stereo detector" NO)
if (BUILD_SA_VISION_STEREODETECTOR)
add_subdirectory (src/c++/vision/components/StereoDetector)
#add_subdirectory (src/c++/vision/components/StereoDetectorLearner)
add_subdirectory (src/c++/vision/components/StereoDetectorKinectLines)
endif (BUILD_SA_VISION_STEREODETECTOR)

option(BUILD_SA_VISION_PLANEPOPOUT "build OLD plane pop-out (needs hardware/stereo)" YES)
if(BUILD_SA_VISION_PLANEPOPOUT)
add_subdirectory (src/c++/vision/components/PlanePopOut)
endif(BUILD_SA_VISION_PLANEPOPOUT)

option(BUILD_SA_VISION_PLANEPOPOUT_PC "build PC plane pop-out (needs hardware/stereo)" NO)
if(BUILD_SA_VISION_PLANEPOPOUT_PC)
add_subdirectory (src/c++/vision/components/PlanePopOutPc)
endif(BUILD_SA_VISION_PLANEPOPOUT_PC)

option(BUILD_SA_VISION_SOIFILTER "build SOI filter" YES)
if(BUILD_SA_VISION_SOIFILTER)
add_subdirectory (src/c++/vision/components/SOIFilter)
endif(BUILD_SA_VISION_SOIFILTER)

# Analyzer functions were moved to SOIFilter in CogX Y3
#option(BUILD_SA_VISION_OBJANALYZER "build Object Analyzer" YES)
#if(BUILD_SA_VISION_OBJANALYZER)
#add_subdirectory (src/c++/vision/components/ObjectAnalyzer)
#endif(BUILD_SA_VISION_OBJANALYZER)

option(BUILD_SA_VISION_VISMEDIATOR "build Visual Mediator" YES)
if(BUILD_SA_VISION_VISMEDIATOR)
add_subdirectory (src/c++/vision/components/VisualMediator)
endif(BUILD_SA_VISION_VISMEDIATOR)

option(BUILD_SA_VISION_VIRTUALSCENE "build 3D Virtual Scene" YES)
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

option(BUILD_SA_VISION_OBJRECOGNIZER_3D3 "build Object Recognizer3 (new version)" NO)
if (BUILD_SA_VISION_OBJRECOGNIZER_3D3)
add_subdirectory (src/c++/vision/components/ObjectRecognizer3D3)
endif (BUILD_SA_VISION_OBJRECOGNIZER_3D3)

option(BUILD_SA_VISION_DISPLAY_CONVEXHULL_PB "build convex hull display" YES)
if (BUILD_SA_VISION_DISPLAY_CONVEXHULL_PB)
add_subdirectory (src/c++/vision/components/DisplayConvexHullPB)
endif (BUILD_SA_VISION_DISPLAY_CONVEXHULL_PB)

include(cmake/VisualLearner.cmake)

option(BUILD_SA_VISION_SIMULATION "Build Simulated Vision" YES)
if(BUILD_SA_VISION_SIMULATION)
   add_subdirectory (src/c++/vision/components/SimulatedVision)
endif(BUILD_SA_VISION_SIMULATION)

option(BUILD_SA_VISION_CAMERA_MOUNT "build camera mount" YES)
if (BUILD_SA_VISION_CAMERA_MOUNT)
add_subdirectory (src/c++/vision/components/CameraMount)
add_subdirectory (src/c++/vision/components/CheckSystemCalibration)
endif (BUILD_SA_VISION_CAMERA_MOUNT)

option(BUILD_SA_VISION_PG_VIEWER "build PointGrey Viewer" YES)
if(BUILD_SA_VISION_PG_VIEWER)
add_subdirectory (src/c++/vision/components/PointGreyViewer)
endif(BUILD_SA_VISION_PG_VIEWER)

option(BUILD_SA_VISION_TEST_COMP "build test component" YES)
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

option(BUILD_SA_VISION_CHECK_SYS_CALIB "build system calibration check" NO)
if (BUILD_SA_VISION_CHECK_SYS_CALIB)
add_subdirectory (src/c++/vision/components/CheckSystemCalibration)
endif (BUILD_SA_VISION_CHECK_SYS_CALIB)

option(BUILD_SA_VISION_GAZEBO "build gazebo simulated vision" NO)
if (BUILD_SA_VISION_GAZEBO)
add_subdirectory (src/c++/vision/components/GazeboVision)
endif (BUILD_SA_VISION_GAZEBO)

option(BUILD_SA_VISION_GAZEBO_JUGGLER "build gazebo object juggler" NO)
if (BUILD_SA_VISION_GAZEBO_JUGGLER)
add_subdirectory (src/c++/vision/components/GazeboJuggler)
endif (BUILD_SA_VISION_GAZEBO_JUGGLER)


option(BUILD_SA_VISION_VIEWCONE_COSTS "build viewcone cost generator" YES)
if (BUILD_SA_VISION_VIEWCONE_COSTS)
add_subdirectory (src/c++/vision/components/ViewConeMoveCostGenerator)
endif (BUILD_SA_VISION_VIEWCONE_COSTS)

if(BUILD_PYTHON_COMPONENTS)
   add_subdirectory (src/python)
endif(BUILD_PYTHON_COMPONENTS)

