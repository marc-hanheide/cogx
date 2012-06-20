cmake_minimum_required(VERSION 2.6)

find_package(PkgConfig REQUIRED)
pkg_search_module(OPENCV REQUIRED opencv)

include_directories(.)
include_directories(${OPENCV_INCLUDE_DIRS})

#set ( OPENCV_LIBRARIES  cv cxcore highgui )
#include_directories("$ENV{HOME}/projects/OpenCV-2.1.0/include")
#SET(OPENCV_LIB_PATH "$ENV{HOME}/projects/OpenCV-2.1.0/build/lib")
#SET ( OPENCV_LIBRARIES  ${OPENCV_LIB_PATH}/libcv.so ${OPENCV_LIB_PATH}/libcxcore.so ${OPENCV_LIB_PATH}/libhighgui.so )

set(CMAKE_CXX_FLAGS "-Wall -Werror -g")

set(Boost_USE_MULTITHREADED ON)
FIND_PACKAGE (Boost 1.38.0 REQUIRED COMPONENTS system date_time thread program_options serialization regex filesystem signals)
set (BOOST_LIBRARIES ${Boost_LIBRARIES})

add_executable(calib_stereo main.cpp)
target_link_libraries(calib_stereo ${OPENCV_LIBRARIES} ${BOOST_LIBRARIES})
include_directories(./ ${AK_INCLUDES})
install (TARGETS calib_stereo DESTINATION $ENV{HOME}/bin)

#add_executable(kinectStereoPoseCalib kinectStereoPoseCalib.cpp)
#target_link_libraries(kinectStereoPoseCalib ${OPENCV_LIBRARIES} ${BOOST_LIBRARIES})
#include_directories(./ ${AK_INCLUDES})
#install (TARGETS kinectStereoPoseCalib DESTINATION $ENV{HOME}/bin)
