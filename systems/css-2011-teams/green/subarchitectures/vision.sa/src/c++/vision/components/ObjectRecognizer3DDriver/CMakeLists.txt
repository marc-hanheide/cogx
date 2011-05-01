include_directories(.)
include_directories(../ObjectTracker)
include_directories(../ObjectTracker/Tracker/include)

include_directories(${COGX_ROOT}/subarchitectures/manipulation.sa/src/c++/autogen)

# Set the sources.
SET(SOURCES ObjectRecognizer3DDriver.cpp)

add_cast_component(ObjectRecognizer3DDriver ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData)
link_cast_component(${CAST_COMPONENT_NAME} Tracker)
link_cast_component(${CAST_COMPONENT_NAME} ManipulationData)

