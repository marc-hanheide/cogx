include_directories(.)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen)

# Set the sources.
SET(SOURCES DummyDriver.cpp)


add_cast_component(DummyDriver ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData)

