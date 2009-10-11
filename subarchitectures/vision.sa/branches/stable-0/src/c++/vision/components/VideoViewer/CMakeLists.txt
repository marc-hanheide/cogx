include_directories(.)

# Set the sources.
SET(SOURCES VideoViewer.cpp)

add_cast_component(VideoViewer ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils ${OPENCV_LDFLAGS})

