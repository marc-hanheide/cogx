include_directories(.)

# Set the sources.
SET(SOURCES SOIFilter.cpp)


add_cast_component(SOIFilter ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData VideoUtils VideoClient)

