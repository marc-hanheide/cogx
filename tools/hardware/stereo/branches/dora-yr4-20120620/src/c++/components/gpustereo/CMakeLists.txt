cmake_minimum_required(VERSION 2.6)

# ########################################################################### 
# SIFT implementation with pure CUDA
# * Matching works.
# * Extraction works, but is not stable; depends on the image and its size.
# ########################################################################### 

project(GpuStereo)

set(OUTDIR ${CMAKE_CURRENT_BINARY_DIR})
set(LIBNAME libgpustereo.so)
SET(CMAKE_CXX_FLAGS ${CXX_FLAGS} "-fPIC")
add_custom_command(
   OUTPUT ${OUTDIR}/${LIBNAME}
   COMMAND make -f ${CMAKE_CURRENT_SOURCE_DIR}/Makefile_cmake lib
      SOURCEDIR="${CMAKE_CURRENT_SOURCE_DIR}" DESTDIR="${OUTDIR}"
   DEPENDS 
      Makefile_cmake
      census.cu census.h CensusGPU.cpp CensusGPU.h
      align.h
   )

add_custom_target(GpuStereo ALL
   DEPENDS ${OUTDIR}/${LIBNAME}
   )

install(FILES ${OUTDIR}/${LIBNAME} DESTINATION lib)
