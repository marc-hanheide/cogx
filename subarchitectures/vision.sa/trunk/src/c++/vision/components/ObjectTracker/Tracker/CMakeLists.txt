cmake_minimum_required(VERSION 2.6)

project(Tracker)

FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE( OpenGL REQUIRED )

add_definitions(-DGL_GLEXT_PROTOTYPES)

include_directories(include)
include_directories(/usr/include/SDL)

link_directories(/usr/local/lib)

# Set the sources
SET(SOURCES
	src/CDataFile.cpp
	src/Camera.cpp
	src/EdgeTracker.cpp
	src/Frustum.cpp
	src/ImageProcessor.cpp
	src/TrackerModel.cpp
	src/Model.cpp
	src/ModelLoader.cpp
	src/myPredictor.cpp
	src/Particle.cpp
	src/Predictor.cpp
	src/Distribution.cpp
	src/ply.c
	src/Pose.cpp
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


add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME} ${OPENCV_LDFLAGS} SDL GLU glut)
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES})

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

