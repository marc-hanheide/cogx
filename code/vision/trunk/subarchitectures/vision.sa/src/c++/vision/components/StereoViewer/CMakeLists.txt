include_directories(.)

add_cast_component(StereoViewer StereoViewer.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Stereo StereoClient)

