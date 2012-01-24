cmake_minimum_required(VERSION 2.6)

PROJECT(VSZTomGine)

add_definitions(-DGL_GLEXT_PROTOTYPES)

# Set the directories
include_directories(./include)
include_directories(/usr/include/GL)
include_directories(/usr/include/freetype2)

#link_directories(/usr/lib)
link_directories(/usr/local/lib)

# Set the sources.
SET(SOURCES
	src/ply.c
	src/tgQuaternion.cpp
	src/tgCamera.cpp
	src/tgEngine.cpp
	src/tgFont.cpp
	src/tgFrustum.cpp
	src/tgLabel.cpp
	src/tgLighting.cpp
	src/tgMatrix3.cpp
	src/tgModel.cpp
	src/tgModelLoader.cpp
	src/tgPose.cpp
	src/tgRenderModel.cpp
	src/tgShapeCreator.cpp
	src/tgTexture.cpp
	src/tgVector3.cpp
	src/tgGLXWindow.cpp
	src/tgTimer.cpp)

LINK_LIBRARIES(X11 GL GLU ftgl)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

