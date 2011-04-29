cmake_minimum_required(VERSION 2.6)

project(siftgpu)

include_directories(../include)
include_directories(/usr/include/Cg)
include_directories(/usr/local/cuda/include)

#add_definitions(-DCUDA_SIFTGPU_ENABLED)

link_directories(/usr/local/cuda/lib)

SET(SOURCES 
		../src/SiftGPU/CuTexImage.cpp
		../src/SiftGPU/FrameBufferObject.cpp
		../src/SiftGPU/GlobalUtil.cpp
		../src/SiftGPU/GLTexImage.cpp
		../src/SiftGPU/ProgramCG.cpp
		../src/SiftGPU/ProgramGLSL.cpp
		../src/SiftGPU/ProgramGPU.cpp
		../src/SiftGPU/PyramidCU.cpp
		../src/SiftGPU/PyramidGL.cpp
		../src/SiftGPU/ShaderMan.cpp
		../src/SiftGPU/SiftGPU.cpp
		../src/SiftGPU/SiftMatch.cpp
		../src/SiftGPU/SiftMatchCU.cpp
		../src/SiftGPU/SiftPyramid.cpp)


add_library(${PROJECT_NAME} SHARED ${SOURCES})
link_libraries(Cg CgGL GLEW glut IL cudart)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
