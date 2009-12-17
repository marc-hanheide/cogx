cmake_minimum_required(VERSION 2.6)

PROJECT(TomGine)

add_definitions(-DGL_GLEXT_PROTOTYPES)

# Set the directories
include_directories(./include)
include_directories(/usr/local/include/opencv)
include_directories(/usr/include/SDL)
include_directories(/usr/include/GL)

link_directories(/usr/lib)
link_directories(/usr/local/lib)

# Set the sources.
SET(SOURCES 
	src/tgCamera.cpp
	src/tgEngine.cpp
	src/tgFrustum.cpp
	src/tgLighting.cpp
	src/tgMatrix3.cpp
	src/tgModel.cpp
	src/tgTexture.cpp
	src/tgVector3.cpp
	src/Timer.cpp)
SET(HEADERS 
	include/BasicGeometries.h
	include/mathlib.h
	include/tgCamera.h
	include/tgEngine.h
	include/tgFrustum.h
	include/tgLighting.h
	include/tgMatrix3.h
	include/tgModel.h
	include/tgTexture.h
	include/tgVector3.h
	include/Timer.h)

LINK_LIBRARIES(GL SDL cv)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)


