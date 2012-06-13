project(vision.ObjectRecognizer3D3)

find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

FIND_PACKAGE( PCL REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

FIND_PACKAGE( V4R REQUIRED )
include_directories(${V4R_INCLUDE_DIRS})
link_directories(${V4R_LIBRARY_DIRS})

include_directories(.)
include_directories(../..)

ENABLE_COGX_VISUALIZATION(TRUE OPENCV)

add_cast_component(ObjectRecognizer3D3 ObjectRecognizer3D.cpp)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils ${VISUALIZATION_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} v4rCModel v4rPCore)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS})

