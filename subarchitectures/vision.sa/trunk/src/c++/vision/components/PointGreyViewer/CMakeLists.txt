include_directories(.)

# Set the sources.
SET(SOURCES PointGreyViewer.cpp)

add_cast_component(PointGreyViewer ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils ${OPENCV_LDFLAGS})

