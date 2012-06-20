include_directories(.)

# Set the sources.
SET(SOURCES PeopleDetectorTest.cpp PeopleDetectorTest.h)

add_cast_component(PeopleDetectorTest ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)

