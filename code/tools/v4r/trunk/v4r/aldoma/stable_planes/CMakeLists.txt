cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(STABLE_PLANES)
SET (CMAKE_BUILD_TYPE Debug)
list(APPEND CMAKE_MODULE_PATH ${STABLE_PLANES_SOURCE_DIR})

find_package(PCL 1.2 REQUIRED)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

OPTION(BUILD_SHARED_LIBS "turn OFF for .a libs" ON)

include_directories(${PCL_INCLUDE_DIRS} include /usr/include/cminpack-1/ /usr/include/eigen3 /usr/local/cuda_4.0/cuda/include/ /home/aitor/pcl/trunk/cuda/common/include)
link_directories(${PCL_LINK_DIRECTORIES})

add_executable(main src/main.cpp)

TARGET_LINK_LIBRARIES(main vtkHybrid vtkCommon vtkIO boost_filesystem pcl_visualization pcl_segmentation pcl_kdtree pcl_surface pcl_filters pcl_features)

add_library(stable_planes_lib src/main.cpp)
target_link_libraries(stable_planes_lib vtkHybrid vtkCommon vtkIO boost_filesystem pcl_visualization pcl_segmentation pcl_kdtree pcl_surface pcl_filters pcl_features)