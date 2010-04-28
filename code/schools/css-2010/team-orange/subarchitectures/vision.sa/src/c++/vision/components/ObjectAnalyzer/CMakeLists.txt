include_directories(.)

# Set the sources.
SET(SOURCES ObjectAnalyzer.cpp)


add_cast_component(ObjectAnalyzer ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData)

