
option(LIBBUILD_SIFTGPU "Build SiftGPU library" ON)
if (LIBBUILD_SIFTGPU)
   add_subdirectory(SiftGPU/linux)
endif(LIBBUILD_SIFTGPU)

option(LIBBUILD_CUDASIFT "Build CudaSift library" ON)
if (LIBBUILD_CUDASIFT)
   add_subdirectory(CudaSift)
endif(LIBBUILD_CUDASIFT)
