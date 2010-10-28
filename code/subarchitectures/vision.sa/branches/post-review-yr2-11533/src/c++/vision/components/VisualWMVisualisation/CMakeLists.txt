include_directories(.)

# Set the sources.
SET(SOURCES VisualWMVisualisation.cpp)

Find_Package(OpenGL)
Find_Package(GLU)
Find_Package(GLUT)

include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})


add_cast_component(VisualWMVisualisation ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData)
link_cast_component(${CAST_COMPONENT_NAME} ${GLUT_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} ${OPENGL_LIBRARIES})

