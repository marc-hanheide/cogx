
include_directories(.)

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})
link_directories(${OPENGL_LIBRARY_DIRS})
find_package(GLU REQUIRED)
include_directories(${GLU_INCLUDE_DIRS})
link_directories(${GLU_LIBRARY_DIRS})
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})

add_cast_component(StereoViewer StereoViewer.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Stereo StereoClient)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENGL_LDFLAGS} ${GLU_LDFLAGS}
    ${GLUT_LDFLAGS} ${OPENCV_LDFLAGS} glut)
