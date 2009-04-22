include_directories(.)

# Set the sources.
SET(SOURCES ObjectTrackerDriver.cpp)


add_cast_component(ObjectTrackerDriver ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData)

