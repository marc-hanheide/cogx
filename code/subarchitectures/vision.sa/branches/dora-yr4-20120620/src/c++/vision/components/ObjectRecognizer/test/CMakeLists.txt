include_directories(.)

# Set the sources.
SET(SOURCES TestRecognizer.cpp TestCases.cpp)

add_cast_component(TestObjectRecognizer ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData VideoUtils ObjectRecognizerSrv)

