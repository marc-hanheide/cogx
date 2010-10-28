cmake_minimum_required(VERSION 2.6)

# ########################################################################### 
# SIFT implementation with pure CUDA
# * Matching works.
# * Extraction works, but is not stable; depends on the image and its size.
# * References to functions from libcutil.a removed.
# ########################################################################### 

project(LibCudaSift)
find_package(Cuda)

macro(nvcc_compile VAR_GENERATED_OBJS VAR_SOURCES VAR_DEPENDENCIES)
   foreach(SRC ${${VAR_SOURCES}})
      set(OBJ "${CMAKE_CURRENT_BINARY_DIR}/${SRC}.o")
      list(APPEND OBJLST ${OBJ})
      set(DEPS ${SRC})
      if (VAR_DEPENDENCIES)
         set(DEPS ${DEPS} ${${VAR_DEPENDENCIES}})
      endif(VAR_DEPENDENCIES)

      add_custom_command(
         OUTPUT ${OBJ}
         COMMAND nvcc 
            -I${CUDA_INSTALLROOT}/include
            -I${CUDA_INSTALLROOT}/sdk/common/inc
            -I${CUDA_INSTALLROOT}/C/include
            -I${CUDA_INSTALLROOT}/sdk/C/common/inc
            -L${CUDA_INSTALLROOT}/lib
            # Cuda 2.2/2.3, gcc 4.4
            #    avoid error: inline function ‘...’ cannot be declared weak
            #    also removed option: -O2
            --compiler-options -fPIC,-fno-inline,-D__builtin_stdarg_start=__builtin_va_start
            -o ${OBJ} -c ${CMAKE_CURRENT_SOURCE_DIR}/${SRC}
         DEPENDS ${DEPS}
         COMMENT "nvcc_compile: Compiling '${SRC}'"
         )
   endforeach(SRC)
   foreach(OBJ ${OBJLST})
      set(${VAR_GENERATED_OBJS} ${${VAR_GENERATED_OBJS}} ${OBJ})
   endforeach(OBJ)
endmacro(nvcc_compile)

set(LIBNAME cudasift)
set(CUDA_SOURCES
   cudaSiftH.cu featureUploadH.cu cudaImage.cu 
   )
set(CUDA_HEADERS
   cudaImage.h cudaSiftD.h cudaSift.h cudaSiftH.h featureUploadH.h tpimage.h tpimageutil.h
   )
set(CUDA_DEPS ${CUDA_HEADERS} cudaSiftD.cu)
nvcc_compile(CUDA_OBJS CUDA_SOURCES CUDA_DEPS)

link_directories(
   ${CUDA_INSTALLROOT}/lib
   )
add_library(${LIBNAME} SHARED
   cudaSift_gold.cpp
   ${CUDA_OBJS}
   )
target_link_libraries(${LIBNAME}
   cudart
   )

install(TARGETS ${LIBNAME} DESTINATION ${COGX_ROOT}/output/lib)

