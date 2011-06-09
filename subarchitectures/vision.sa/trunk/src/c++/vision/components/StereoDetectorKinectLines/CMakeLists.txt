include_directories(.)

add_definitions(-DHAVE_CAST)

# include openNI headers for kinect sensor
include_directories(/usr/include/ni)
#include_directories(Kinect)

include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/TomGine)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/TomGine/TomGine)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/TomGine/TomGineWraper)

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


find_package(PCL 1.0 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})

# Set the sources.
SET(HEADERS
	StereoDetectorKinectLines.h
)
SET(SOURCES 
	StereoDetectorKinectLines.cpp
)
add_cast_component(StereoDetectorKinectLines ${SOURCES} ${HEADERS})

link_cast_component(${CAST_COMPONENT_NAME} SDGestalts3D SDUtils SDObjRepresentation SDStereo SDMatas SDvs3 SDCEdge SDKinect SDPCL SDLearner)
link_cast_component(${CAST_COMPONENT_NAME} freetype ftgl GLWindow TomGine TomGineWraper)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)
link_cast_component(${CAST_COMPONENT_NAME} ${PCL_LIBRARIES})


