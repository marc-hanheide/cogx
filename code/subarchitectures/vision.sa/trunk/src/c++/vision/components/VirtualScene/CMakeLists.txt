project(VirtualScene)

include_directories(.)
include_directories(TomGine/include)
include_directories(/usr/include/opencv)

add_subdirectory(TomGine)

link_directories(/usr/local/lib)

# Set the sources
SET(SOURCES VirtualScene.cpp)

SET(HEADERS ModelEntry.h VirtualScene.h)


add_cast_component(VirtualScene ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils TomGine)
link_cast_component(${CAST_COMPONENT_NAME} GLU)
