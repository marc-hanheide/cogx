cmake_minimum_required(VERSION 2.6)
project(PythonSiftGpu)

find_package(PythonLibs)

if (LIBBUILD_CUDASIFT)
   find_package(Cuda)

   link_directories(
      ${CUDA_INSTALLROOT}/lib
   )
endif (LIBBUILD_CUDASIFT)


#set(SIFTGPU_SRCDIR ../../libsiftgpu/SiftGPU)
#include_directories(
#   ${SIFTGPU_SRCDIR}/src
#   ${SIFTGPU_SRCDIR}/../Include
#   /usr/include/GL
#   ${PYTHON_INCLUDE_PATH}
#   )

#set(LIB_SOURCES_GLSL
#   ${SIFTGPU_SRCDIR}/src/FrameBufferObject.cpp
#   ${SIFTGPU_SRCDIR}/src/GlobalUtil.cpp
#   ${SIFTGPU_SRCDIR}/src/GLTexImage.cpp
#   ${SIFTGPU_SRCDIR}/src/ProgramCG.cpp
#   ${SIFTGPU_SRCDIR}/src/ProgramGLSL.cpp
#   ${SIFTGPU_SRCDIR}/src/ProgramGPU.cpp
#   ${SIFTGPU_SRCDIR}/src/ShaderMan.cpp
#   ${SIFTGPU_SRCDIR}/src/SiftGPU.cpp
#   ${SIFTGPU_SRCDIR}/src/SiftPyramid.cpp
#   ${SIFTGPU_SRCDIR}/src/PyramidGL.cpp
#   ${SIFTGPU_SRCDIR}/src/SiftMatch.cpp
#   )

#set(LIB_SOURCES_CUDA
#   ${SIFTGPU_SRCDIR}/src/FrameBufferObject.cpp
#   ${SIFTGPU_SRCDIR}/src/GlobalUtil.cpp
#   ${SIFTGPU_SRCDIR}/src/GLTexImage.cpp
#   ${SIFTGPU_SRCDIR}/src/ProgramCG.cpp
#   ${SIFTGPU_SRCDIR}/src/ProgramGLSL.cpp
#   ${SIFTGPU_SRCDIR}/src/ProgramGPU.cpp
#   ${SIFTGPU_SRCDIR}/src/ShaderMan.cpp
#   ${SIFTGPU_SRCDIR}/src/SiftGPU.cpp
#   ${SIFTGPU_SRCDIR}/src/SiftPyramid.cpp
#   ${SIFTGPU_SRCDIR}/src/PyramidGL.cpp
#   ${SIFTGPU_SRCDIR}/src/SiftMatch.cpp
#   ${SIFTGPU_SRCDIR}/src/CuTexImage.cpp
#   ${SIFTGPU_SRCDIR}/src/PyramidCU.cpp
#   ${SIFTGPU_SRCDIR}/src/SiftMatchCU.cpp
#   )

## TODO: option to select GLSL or CUDA sources
#set(LIB_SOURCES ${LIB_SOURCES_GLSL})

include_directories(
  ${PYTHON_INCLUDE_PATH}
  /usr/include/GL
  ../../../../libs/SiftGPU/src/SiftGPU
  )

add_library(PySiftGpu SHARED
   ${LIB_SOURCES}
   pysiftgpu.cpp
   )
add_dependencies(PySiftGpu siftgpu)

# SET(CMAKE_CXX_FLAGS ${CXX_FLAGS} "-Wall -Wno-deprecated -pthread -mfpmath=sse")
target_link_libraries(PySiftGpu
   Cg CgGL GLEW IL
   siftgpu
   # glut
   )
set_target_properties(PySiftGpu PROPERTIES 
   PREFIX ""
   OUTPUT_NAME "siftgpu"
   COMPILE_FLAGS "-Wall -Wno-deprecated -pthread -mfpmath=sse"
   )
install(TARGETS PySiftGpu DESTINATION ${COGX_ROOT}/output/python)


