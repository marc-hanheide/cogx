include_directories(.)

# Set the sources.
SET(SOURCES DummyDriver.cpp)


add_cast_component(DummyDriver ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData)

