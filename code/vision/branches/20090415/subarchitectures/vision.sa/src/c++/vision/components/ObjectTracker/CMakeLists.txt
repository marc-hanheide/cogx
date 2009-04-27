project(ObjectTracker)

include_directories(.)
include_directories(include)

include_directories(/usr/local/include/opencv)
include_directories(/usr/include/SDL)

link_directories(/usr/local/lib)

add_definitions(-DGL_GLEXT_PROTOTYPES)

# Set the sources
SET(SRC src/Camera.cpp src/Frustum.cpp src/ImageProcessor.cpp src/TM_Matrix3.cpp src/Model.cpp src/OpenGLControl.cpp src/Particles.cpp src/ply.c src/PlyModel.cpp src/Resources.cpp src/Shader.cpp src/ShaderUtils.cpp src/Texture.cpp src/Timer.cpp src/Tracker.cpp src/TM_Vector3.cpp)
#SET(SRC src/*.cpp)
SET(SOURCES ObjectTracker.cpp ${SRC})


add_cast_component(ObjectTracker ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils VideoClient)
link_cast_component(${CAST_COMPONENT_NAME} cv highgui SDL GL glut)
