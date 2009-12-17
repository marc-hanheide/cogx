project(VirtualScene)

include_directories(.)
include_directories(TomGine/include)

add_subdirectory(TomGine)

# Set the sources
SET(SOURCES VirtualScene.cpp)


add_cast_component(VirtualScene ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils TomGine)
link_cast_component(${CAST_COMPONENT_NAME} cv SDL GLU glut)