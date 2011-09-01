cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(alignment)
SET (CMAKE_BUILD_TYPE Debug)
list(APPEND CMAKE_MODULE_PATH ${alignment_SOURCE_DIR})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PCL 1.2 REQUIRED)

INCLUDE_DIRECTORIES(
   include /home/aitor/v4r_svn/aldoma ${PCL_INCLUDE_DIRS} /home/aitor/v4r_svn/aldoma/stable_planes/include /home/aitor/NVIDIA_GPU_Computing_SDK/C/common/inc/
)

LINK_DIRECTORIES(${PCL_LINK_DIRECTORIES} /home/aitor/v4r_svn/aldoma/stable_planes/build)

OPTION(BUILD_SHARED_LIBS "turn OFF for .a libs" ON)

#Link with libcutil (expected to be in ${CUDA_SDK_ROOT_DIR}/C/lib)
#LINK_LIBRARIES("-L/${CUDA_SDK_ROOT_DIR}/C/lib -lcutil_i386 -L/usr/local/cuda/lib -lcufft")

#Include FindCUDA script
INCLUDE(FindCUDA)

find_package( OpenCV REQUIRED )

set(CMAKE_CXX_FLAGS -fopenmp)

#common commands for building c++ executables and libraries
cuda_add_library(${PROJECT_NAME} src/stable_planes_alignment.cpp include/alignment/logPolarTranslation.cu)
target_link_libraries(${PROJECT_NAME} boost_filesystem fftw3 vtkRendering vtkGraphics vtkImaging vtkFiltering vtkCommon vtkHybrid pcl_visualization pcl_kdtree pcl_surface pcl_filters pcl_segmentation gomp ${OpenCV_LIBS})

add_executable(align src/align.cpp)
target_link_libraries(align stable_planes_lib boost_filesystem vtkRendering vtkGraphics vtkImaging vtkFiltering vtkCommon vtkHybrid pcl_visualization pcl_kdtree pcl_surface pcl_filters pcl_segmentation gomp ${PROJECT_NAME})