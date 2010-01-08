cmake_minimum_required(VERSION 2.6)

project(Tracker)

add_definitions(-DGL_GLEXT_PROTOTYPES)

include_directories(include)
#include_directories(../ObjectDetector/mxCvTools)
include_directories(/usr/include/opencv)
include_directories(/usr/include/SDL)

link_directories(/usr/local/lib)

# Set the sources
SET(SOURCES
	src/CDataFile.cpp
	src/Camera.cpp
	src/EdgeTracker.cpp
	src/Frustum.cpp
	src/ImageProcessor.cpp
	src/Model.cpp
	src/Particles.cpp
	src/ply.c
	src/PlyModel.cpp
	src/Quaternion.cpp
	src/Resources.cpp
	src/Shader.cpp
	src/ShaderUtils.cpp
	src/Texture.cpp
	src/TextureTracker.cpp
	src/tgLighting.cpp
	src/Timer.cpp
	src/TM_Matrix3.cpp
	src/TM_Vector3.cpp
	src/Tracker.cpp)

link_libraries(cv highgui SDL GLU glut)
add_library(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

