include_directories(.)

add_cast_component(StereoViewer StereoViewer.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Stereo StereoClient)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} GLU glut ${OPENCV_LDFLAGS})
