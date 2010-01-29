
project(siftgpu)

include_directories(../Include)
include_directories(/usr/include/Cg)


SET(SOURCES 
		../SiftGPU/src/FrameBufferObject.cpp
		../SiftGPU/src/GlobalUtil.cpp
		../SiftGPU/src/GLTexImage.cpp
		../SiftGPU/src/ProgramCG.cpp
		../SiftGPU/src/ProgramGLSL.cpp
		../SiftGPU/src/ProgramGPU.cpp
		../SiftGPU/src/ShaderMan.cpp
		../SiftGPU/src/SiftGPU.cpp
		../SiftGPU/src/SiftMatch.cpp
		../SiftGPU/src/SiftPyramid.cpp)

link_libraries(Cg CgGL GLEW glut IL)
add_library(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
