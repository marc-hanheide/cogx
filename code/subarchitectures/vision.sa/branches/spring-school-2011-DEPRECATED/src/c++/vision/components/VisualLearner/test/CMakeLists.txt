include_directories(.)

# Set the sources.
SET(SOURCES TestLearner.cpp TestCases.cpp)

add_cast_component(TestVisualLearner ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData VideoUtils)

