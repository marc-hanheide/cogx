
project(siftgpu)

include_directories(../include)

find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_PATH})

find_package(Cg REQUIRED)
include_directories(${CG_INCLUDE_PATH})

find_package(X11 REQUIRED)

find_package(IL REQUIRED)
include_directories(${IL_INCLUDE_DIRS})

find_package(OpenGL REQUIRED)

if(LIBBUILD_SIFTGPU_WITH_CUDA)
   find_package(Cuda)
   add_definitions(-DCUDA_SIFTGPU_ENABLED)
   include_directories(${CUDA_INSTALLROOT}/include)
   link_directories(${CUDA_INSTALLROOT}/lib)
   set(CUDA_SOURCES
      ../src/SiftGPU/CuTexImage.cpp
      ../src/SiftGPU/SiftMatchCU.cpp
      ../src/SiftGPU/PyramidCU.cpp
      )
   set(CUDA_LIBRARIES
      cudart
      )
   set(SRC_PROGRAM_CU ${CMAKE_CURRENT_SOURCE_DIR}/../src/SiftGPU/ProgramCU.cu)
   set(OBJ_PROGRAM_CU ${CMAKE_CURRENT_BINARY_DIR}/ProgramCU.so)
   add_custom_command(
      OUTPUT ${OBJ_PROGRAM_CU}
      COMMAND nvcc -c 
	 -I${CUDA_INSTALLROOT}/include -L${CUDA_INSTALLROOT}/lib -DCUDA_SIFTGPU_ENABLED
	 # Cuda 2.2/2.3, gcc 4.4
	 #    avoid error: inline function ‘...’ cannot be declared weak
	 #    also removed option: -O2
	 --compiler-options -fPIC,-fno-inline,-D__builtin_stdarg_start=__builtin_va_start
	 -o ${OBJ_PROGRAM_CU} ${SRC_PROGRAM_CU}
      DEPENDS ${SRC_PROGRAM_CU}
      COMMENT "SiftProgramCU: Compiling CUDA code with nvcc"
      )
   add_custom_target(SiftProgramCU
      DEPENDS ${OBJ_PROGRAM_CU}
      )
endif(LIBBUILD_SIFTGPU_WITH_CUDA)

SET(SOURCES 
   ../src/SiftGPU/FrameBufferObject.cpp
   ../src/SiftGPU/GlobalUtil.cpp
   ../src/SiftGPU/GLTexImage.cpp
   ../src/SiftGPU/ProgramCG.cpp
   ../src/SiftGPU/ProgramGLSL.cpp
   ../src/SiftGPU/ProgramGPU.cpp
   ../src/SiftGPU/PyramidGL.cpp
   ../src/SiftGPU/ShaderMan.cpp
   ../src/SiftGPU/SiftGPU.cpp
   ../src/SiftGPU/SiftMatch.cpp
   ../src/SiftGPU/SiftPyramid.cpp
   ${CUDA_SOURCES}
   )

set(LIBRARY_NAME ${PROJECT_NAME})

add_library(${LIBRARY_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME} ${X11_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${CG_LIBRARY} ${CG_GL_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${GLEW_LIBRARY})	
target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES})	
target_link_libraries(${PROJECT_NAME} ${IL_LIBRARIES})	
# why does this not work here? target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES})
# have to do this instead:
target_link_libraries(${PROJECT_NAME} GL)


if (LIBBUILD_SIFTGPU_WITH_CUDA)
   add_dependencies(${LIBRARY_NAME} SiftProgramCU)
   set_target_properties(${LIBRARY_NAME}
      PROPERTIES
      LINK_FLAGS ${OBJ_PROGRAM_CU}
      )
endif(LIBBUILD_SIFTGPU_WITH_CUDA)

INSTALL(TARGETS ${LIBRARY_NAME} LIBRARY DESTINATION lib)
