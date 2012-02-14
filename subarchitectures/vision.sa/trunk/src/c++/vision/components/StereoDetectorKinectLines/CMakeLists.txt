include_directories(.)

add_definitions(-DHAVE_CAST)
add_definitions(-DLINUX)

# include openNI headers for kinect sensor
include_directories(/usr/include/ni)

include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/StereoDetector/pcl)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/StereoDetector/matas)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/StereoDetector/math)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/StereoDetector/utils)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/StereoDetector/Gestalts3D)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/StereoDetector/vs3)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/StereoDetector/stereo)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/StereoDetector/reasoner)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/StereoDetector/CEdge)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/StereoDetector/objRepresentation)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/StereoDetector/kinect)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/StereoDetector/Learner)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/StereoDetector/GraphCut)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/StereoDetector/libSVM)

FIND_PACKAGE (OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

# Only if v4r libraries are available!
option(HAVE_V4R "TUW v4r library is available" NO)
if (HAVE_V4R)
include(${COGX_ROOT}/cmake/FindV4R.cmake)
include_directories(${V4R_DIR})
link_directories(${V4R_DIR}/lib)

find_package(PCL REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Set the sources.
SET(HEADERS
  StereoDetectorKinectLines.h
)
SET(SOURCES 
  StereoDetectorKinectLines.cpp
)

add_cast_component(SegLearner SegUtilsFunctions.cpp SegLearner.cpp SegLearner.h)
#link_cast_component(${CAST_COMPONENT_NAME} SDGestalts3D SDUtils SDObjRepresentation SDStereo SDMatas SDvs3 SDCEdge SDKinect SDLearner SDGraphCut SDlibSVM)
#link_cast_component(${CAST_COMPONENT_NAME} freetype ftgl)
link_cast_component(${CAST_COMPONENT_NAME} SDUtils SDvs3)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)
link_cast_component(${CAST_COMPONENT_NAME} ${OpenCV_LIBS} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} v4rPCLAddOns v4rsvm v4rGraphCut v4rSurfaceModeling v4rAnnotation v4rNurbsConvertion)

add_cast_component(SegLearner2ndLevel SegUtilsFunctions.cpp SegLearner2ndLevel.cpp SegLearner2ndLevel.h)
#link_cast_component(${CAST_COMPONENT_NAME} SDGestalts3D SDUtils SDObjRepresentation SDStereo SDMatas SDvs3 SDCEdge SDKinect SDLearner SDGraphCut SDlibSVM)
#link_cast_component(${CAST_COMPONENT_NAME} freetype ftgl)
link_cast_component(${CAST_COMPONENT_NAME} SDUtils SDvs3)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)
link_cast_component(${CAST_COMPONENT_NAME} ${OpenCV_LIBS} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} v4rPCLAddOns v4rsvm v4rGraphCut v4rSurfaceModeling v4rAnnotation v4rNurbsConvertion)

add_cast_component(SegTester SegUtilsFunctions.cpp SegTester.cpp SegTester.h)
#link_cast_component(${CAST_COMPONENT_NAME} SDGestalts3D SDUtils SDObjRepresentation SDStereo SDMatas SDvs3 SDCEdge SDKinect SDLearner SDGraphCut SDlibSVM)
#link_cast_component(${CAST_COMPONENT_NAME} freetype ftgl)
link_cast_component(${CAST_COMPONENT_NAME} SDUtils SDvs3)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)
link_cast_component(${CAST_COMPONENT_NAME} ${OpenCV_LIBS} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} v4rPCLAddOns v4rsvm v4rGraphCut v4rSurfaceModeling v4rAnnotation v4rNurbsConvertion)

add_cast_component(SegTester2ndLevel SegUtilsFunctions.cpp SegTester2ndLevel.cpp SegTester2ndLevel.h)
#link_cast_component(${CAST_COMPONENT_NAME} SDGestalts3D SDUtils SDObjRepresentation SDStereo SDMatas SDvs3 SDCEdge SDKinect SDLearner SDGraphCut SDlibSVM)
#link_cast_component(${CAST_COMPONENT_NAME} freetype ftgl)
link_cast_component(${CAST_COMPONENT_NAME} SDUtils SDvs3)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)
link_cast_component(${CAST_COMPONENT_NAME} ${OpenCV_LIBS} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} v4rPCLAddOns v4rsvm v4rGraphCut v4rSurfaceModeling v4rAnnotation v4rNurbsConvertion)

add_cast_component(Segmenter SegUtilsFunctions.cpp Segmenter.cpp Segmenter.h)
#link_cast_component(${CAST_COMPONENT_NAME} SDGestalts3D SDUtils SDObjRepresentation SDStereo SDMatas SDvs3 SDCEdge SDKinect SDLearner SDGraphCut SDlibSVM)
#link_cast_component(${CAST_COMPONENT_NAME} freetype ftgl)
link_cast_component(${CAST_COMPONENT_NAME} SDUtils SDvs3)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)
link_cast_component(${CAST_COMPONENT_NAME} ${OpenCV_LIBS} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} v4rPCLAddOns v4rsvm v4rGraphCut v4rSurfaceModeling v4rAnnotation v4rNurbsConvertion)

endif (HAVE_V4R)
