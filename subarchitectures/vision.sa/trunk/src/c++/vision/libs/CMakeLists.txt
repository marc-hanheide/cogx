
option(LIBBUILD_SIFTGPU "Build SiftGPU library" ON)
if (LIBBUILD_SIFTGPU)
   add_subdirectory(SiftGPU/linux)
endif(LIBBUILD_SIFTGPU)


