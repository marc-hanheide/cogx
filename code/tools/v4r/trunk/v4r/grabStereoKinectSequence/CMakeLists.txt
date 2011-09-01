SET(V4R_GRAB_STEREO_KINECT 0 CACHE BOOL "Grab image & point-cloud sequences from stereo and kinect cam." )
IF(V4R_GRAB_STEREO_KINECT)

cmake_minimum_required (VERSION 2.6)

find_package(PkgConfig REQUIRED)
pkg_search_module(OPENCV REQUIRED opencv)

include_directories(.)
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(/usr/include/ni)

add_library(Kinect SHARED Capture.cpp Device.cpp Kinect.cpp)
target_link_libraries(Kinect OpenNI)

add_executable(grabKinectSequence grabKinectSequence.cpp)
target_link_libraries (grabKinectSequence ${OPENCV_LIBRARIES} Kinect)

add_executable(grabStereoKinectSequence grabStereoKinectSequence.cpp)
target_link_libraries (grabStereoKinectSequence ${OPENCV_LIBRARIES} Kinect)

ENDIF(V4R_GRAB_STEREO_KINECT)
