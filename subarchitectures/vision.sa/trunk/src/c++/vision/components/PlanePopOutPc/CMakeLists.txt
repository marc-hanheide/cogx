FIND_PACKAGE( V4R REQUIRED )
#include_directories(${V4R_DIR})
#link_directories(${V4R_DIR}/lib)


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
include_directories(${COGX_ROOT}/subarchitectures/nav.sa/src/c++/autogen)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/TomGine)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/TomGine/TomGine)

# remove_definitions(-DFEAT_VISUALIZATION)
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

#find_package(OpenGL REQUIRED)
#include_directories(${OPENGL_INCLUDE_DIR})
#link_directories(${OPENGL_LIBRARY_DIRS})

#find_package(GLU REQUIRED)
#include_directories(${GLU_INCLUDE_DIR})
#link_directories(${GLU_LIBRARY_DIRS})

#find_package(GLUT REQUIRED)
#include_directories(${GLUT_INCLUDE_DIR})
#link_directories(${GLUT_LIBRARY_DIRS})

include_directories(${VISUALIZATION_INCLUDE_DIRS})

SET(HEADERS
	PlanePopOut.h
)

SET(SOURCES 
	PlanePopOut.cpp
)

add_cast_component(PlanePopOutPc ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} GLWindow TomGine TomGineWraper) #freetype ftgl
link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)
#link_cast_component(${CAST_COMPONENT_NAME} GLU glut)
link_cast_component(${CAST_COMPONENT_NAME} VisionData VideoUtils Video)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENGL_LDFLAGS} ${GLU_LDFLAGS}
    ${GLUT_LDFLAGS} ${VISUALIZATION_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} v4rpcladdons)
link_cast_component(${CAST_COMPONENT_NAME} pcl_segmentation)

