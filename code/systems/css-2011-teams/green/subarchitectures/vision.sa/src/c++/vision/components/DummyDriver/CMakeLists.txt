include_directories(.)

# Set the sources.
SET(SOURCES DummyDriver.cpp)

include_directories(${COGX_ROOT}/subarchitectures/manipulation.sa/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/ptz/src/c++/autogen)

add_cast_component(DummyDriver ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData)
link_cast_component(${CAST_COMPONENT_NAME} ManipulationData PTZ)
