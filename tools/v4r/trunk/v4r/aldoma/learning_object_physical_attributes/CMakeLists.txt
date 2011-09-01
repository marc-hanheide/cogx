cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(LEARN_ATTRIBUTES)
SET (CMAKE_BUILD_TYPE Debug)
list(APPEND CMAKE_MODULE_PATH ${LEARN_ATTRIBUTES_SOURCE_DIR})

find_package(PCL 1.2 REQUIRED)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

set(OpenCV_DIR /home/aitor/Downloads/OpenCV-2.3.0/build)
find_package( OpenCV REQUIRED )

include_directories(${PCL_INCLUDE_DIRS} include /home/aitor/v4r_svn/aldoma/alignment/include  /home/aitor/v4r_svn/aldoma/stable_planes/include /home/aitor/v4r_svn/aldoma/generic_trainer_recognizer/include /usr/include/cminpack-1/ /usr/include/eigen3)
link_directories(${PCL_LINK_DIRECTORIES} /home/aitor/v4r_svn/aldoma/generic_trainer_recognizer/build  /home/aitor/v4r_svn/aldoma/alignment/build)
add_definitions(${PCL_DEFINITIONS})

OPTION(BUILD_SHARED_LIBS "turn OFF for .a libs" ON)

add_executable(learn_object_attributes src/test.cpp)
target_link_libraries(learn_object_attributes alignment trainer_recognizer pcl_apps)