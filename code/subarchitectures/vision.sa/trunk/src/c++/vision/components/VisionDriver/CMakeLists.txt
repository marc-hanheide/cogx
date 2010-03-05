include_directories(.)

# Set the sources.
SET(SOURCES VisionDriver.cpp)

add_cast_component(VisionDriver ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData)
link_cast_component(${CAST_COMPONENT_NAME} highgui)
