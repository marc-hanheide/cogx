include_directories(.)

# Set the sources.
SET(SOURCES DisplayConvexHullPB.cpp)

add_cast_component(DisplayConvexHullPB ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData PTZ ${OPENCV_LDFLAGS})

