cmake_minimum_required (VERSION 2.6)

find_package(PkgConfig REQUIRED)
pkg_search_module(OPENCV REQUIRED opencv)

find_package(PkgConfig REQUIRED)
pkg_search_module(OPENNI REQUIRED openni-dev)
include_directories(.)
include_directories(${OPENNI_INCLUDE_DIRS})

add_library(Kinect SHARED Capture.cpp Device.cpp Kinect.cpp)
target_link_libraries(Kinect OpenNI)

add_executable(grabStereoKinectSequence FileReaderWriter.h grabStereoKinectSequence.cpp)
target_link_libraries (grabStereoKinectSequence ${OPENCV_LIBRARIES} Kinect)

add_executable(grabStereoKinectSequenceFlyCap FileReaderWriter.h grabStereoKinectSequenceFlyCap.cpp)
target_link_libraries (grabStereoKinectSequenceFlyCap ${OPENCV_LIBRARIES} Kinect flycapture)

