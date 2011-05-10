project(VirtualScene)

include_directories(.)
include_directories(TomGine/include)

FIND_PACKAGE( Freetype REQUIRED )
INCLUDE_DIRECTORIES(${FREETYPE_INCLUDE_DIRS})

add_subdirectory(TomGine)

link_directories(/usr/local/lib)

# Set the sources
SET(SOURCES VirtualScene.cpp)

SET(HEADERS ModelEntry.h VirtualScene.h)

add_cast_component(VirtualScene ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VSZTomGine)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} GLU ftgl)
