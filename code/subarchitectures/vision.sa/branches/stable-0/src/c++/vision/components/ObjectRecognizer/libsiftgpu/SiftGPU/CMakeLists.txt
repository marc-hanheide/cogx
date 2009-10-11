cmake_minimum_required(VERSION 2.6)
project(libsiftgpu)
include_directories(../Include)

add_library(siftgpu SHARED
   src/FrameBufferObject.cpp
   src/GlobalUtil.cpp
   src/GLTexImage.cpp
   src/ProgramCG.cpp
   src/ProgramGLSL.cpp
   src/ProgramGPU.cpp
   src/ShaderMan.cpp
   src/SiftGPU.cpp
   src/SiftPyramid.cpp
   src/PyramidGL.cpp
   src/SiftMatch.cpp
   )
SET(CMAKE_CXX_FLAGS ${CXX_FLAGS} "-Wall -Wno-deprecated -pthread -mfpmath=sse")
target_link_libraries(siftgpu Cg CgGL GLEW glut IL)
install(TARGETS siftgpu DESTINATION ${COGX_ROOT}/output/lib)

