include_directories(.)

add_cast_component(StereoViewer StereoViewer.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Stereo StereoClient)
link_cast_component(${CAST_COMPONENT_NAME} GLU glut)
link_cast_component(${CAST_COMPONENT_NAME} VisionData VideoUtils VideoClient ${OPENCV_LDFLAGS})
