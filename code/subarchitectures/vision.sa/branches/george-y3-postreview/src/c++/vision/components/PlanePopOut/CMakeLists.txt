include_directories(.)
include_directories(${COGX_ROOT}/subarchitectures/nav.sa/src/c++/autogen)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/TomGine)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/TomGine/TomGine)

# remove_definitions(-DFEAT_VISUALIZATION)
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
link_directories(${OPENGL_LIBRARY_DIRS})

find_package(GLU REQUIRED)
include_directories(${GLU_INCLUDE_DIR})
link_directories(${GLU_LIBRARY_DIRS})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})
link_directories(${GLUT_LIBRARY_DIRS})

include_directories(${VISUALIZATION_INCLUDE_DIRS})

SET(HEADERS
	OpenSURF/fasthessian.h
	OpenSURF/integral.h
	OpenSURF/ipoint.h
	OpenSURF/kmeans.h
	OpenSURF/responselayer.h
	OpenSURF/surf.h
	OpenSURF/surflib.h
	OpenSURF/utils.h
	PlanePopOut.h
)

SET(SOURCES 
	OpenSURF/fasthessian.cpp
	OpenSURF/integral.cpp
	OpenSURF/ipoint.cpp
	OpenSURF/surf.cpp
	OpenSURF/utils.cpp
	PlanePopOut.cpp
)

add_cast_component(PlanePopOut ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} freetype ftgl GLWindow TomGine TomGineWraper)
link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)
link_cast_component(${CAST_COMPONENT_NAME} GLU glut)
link_cast_component(${CAST_COMPONENT_NAME} VisionData VideoUtils Video)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENGL_LDFLAGS} ${GLU_LDFLAGS}
    ${GLUT_LDFLAGS} ${VISUALIZATION_LIBRARIES})

