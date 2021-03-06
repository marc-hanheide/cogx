
option(LIBBUILD_SIFTGPU "Build SiftGPU library" ON)
if (LIBBUILD_SIFTGPU)
   option(LIBBUILD_SIFTGPU_WITH_CUDA "Build SiftGPU library with CUDA support" OFF)
   add_subdirectory(SiftGPU/linux)
endif(LIBBUILD_SIFTGPU)

option(LIBBUILD_CUDASIFT "Build CudaSift library" OFF)
if (LIBBUILD_CUDASIFT)
   add_subdirectory(CudaSift)
endif(LIBBUILD_CUDASIFT)

option(LIBBUILD_PYTHON_VISION "Build Python interfaces for some of the libs used by vision.sa" OFF)
if (LIBBUILD_PYTHON_VISION)
   include_directories(.)
   if (LIBBUILD_SIFTGPU)
      include_directories(SiftGPU/src)
      add_subdirectory(python/SiftGPU)
   endif(LIBBUILD_SIFTGPU)
   if (LIBBUILD_CUDASIFT)
      add_subdirectory(python/CudaSift)
   endif(LIBBUILD_CUDASIFT)
endif(LIBBUILD_PYTHON_VISION)

option(LIBBUILD_LEVMAR_HOMEST "Build homest & levmar libraries required by the 3D shape detector" ON)
if (LIBBUILD_LEVMAR_HOMEST)
   option(LIBBUILD_LEVMAR_HOMEST_DEMO "Build demos for homest & levmar" OFF)
   # some prerequisites: liblapack-dev, libf2c2-dev && dependencies
   add_subdirectory(levmar/levmar-2.5)
   add_subdirectory(homest/homest-1.3)
endif (LIBBUILD_LEVMAR_HOMEST)
