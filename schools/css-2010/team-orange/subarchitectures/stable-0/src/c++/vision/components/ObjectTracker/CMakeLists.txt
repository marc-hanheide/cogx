project(ObjectTracker)

include_directories(.)
include_directories(include)

include_directories(../ObjectDetector/mxCvTools)

include_directories(/usr/include/opencv)
include_directories(/usr/include/SDL)

link_directories(/usr/local/lib)

add_definitions(-DGL_GLEXT_PROTOTYPES)

# Set the sources
SET(SRC
	src/Camera.cpp
	src/EdgeTracker.cpp
	src/Frustum.cpp
	src/ImageProcessor.cpp
	src/Kalman.cpp
	src/Lighting.cpp
	src/Model.cpp
	src/OpenGLControl.cpp
	src/Particles.cpp
	src/ply.c
	src/PlyModel.cpp
	src/Quaternion.cpp
	src/Resources.cpp
	src/Shader.cpp
	src/ShaderUtils.cpp
	src/Texture.cpp
	src/TextureTracker.cpp
	src/Timer.cpp
	src/TM_Matrix3.cpp
	src/TM_Vector3.cpp
	src/Tracker.cpp)
SET(SOURCES ObjectTracker.cpp ${SRC})


add_cast_component(ObjectTracker ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} cv highgui SDL GLU glut)
link_cast_component(${CAST_COMPONENT_NAME} ObjectDetector)
