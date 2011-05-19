include_directories(.)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

# Set the sources.
SET(SOURCES ObjectAnalyzer.cpp VTypeMapper.cpp)

add_cast_component(ObjectAnalyzer ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData boost_regex-mt )

