cmake_minimum_required(VERSION 2.6)

# ########################################################################### 
# SIFT implementation with pure CUDA
# * Matching works.
# * Extraction works, but is not stable; depends on the image and its size.
# ########################################################################### 

project(PythonSiftCuda)
find_package(PythonLibs)

set(LIBNAME PyCudaSift)
include_directories(
  ${PYTHON_INCLUDE_PATH}
  /usr/include/GL
  ../../../../libs/CudaSift
  )
link_directories(
   ${CUDA_INSTALLROOT}/lib
   )

add_library(${LIBNAME} SHARED
   pysiftcuda.cpp
   )
add_dependencies(${LIBNAME} siftgpu)

target_link_libraries(${LIBNAME}
   cudasift
   )
set_target_properties(${LIBNAME} PROPERTIES 
   PREFIX ""
   OUTPUT_NAME "siftcuda"
   COMPILE_FLAGS "-Wall -Wno-deprecated -pthread -mfpmath=sse"
   )
install(TARGETS ${LIBNAME} DESTINATION ${COGX_ROOT}/output/python)
