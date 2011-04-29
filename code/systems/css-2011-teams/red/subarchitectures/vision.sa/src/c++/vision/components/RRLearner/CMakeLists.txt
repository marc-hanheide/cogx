include_directories(.)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs/src/cpp/autogen)

# Set the sources.
SET(SOURCES RRLearner.cpp)
 
add_cast_component(RRLearner ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} Beliefs)

