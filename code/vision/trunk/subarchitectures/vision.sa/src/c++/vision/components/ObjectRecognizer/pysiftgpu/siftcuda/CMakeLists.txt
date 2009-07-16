cmake_minimum_required(VERSION 2.6)

# ########################################################################### 
# SIFT implementation with pure CUDA
# * Matching works.
# * Extraction works, but is not stable; depends on the image and its size.
# ########################################################################### 

project(PythonSiftCuda)
find_package(PythonLibs)

set(OUTDIR ${CMAKE_CURRENT_BINARY_DIR})
set(LIBNAME siftcuda.so)
add_custom_command(
   OUTPUT ${OUTDIR}/${LIBNAME}
   COMMAND make -f ${CMAKE_CURRENT_SOURCE_DIR}/Makefile 
      SOURCEDIR="${CMAKE_CURRENT_SOURCE_DIR}" DESTDIR="${OUTDIR}"
   DEPENDS 
      cudaImage.h cudaSiftD.h cudaSift.h cudaSiftH.h featureUploadH.h tpimage.h tpimageutil.h
      pysiftcuda.cpp 
      cudaImage.cu cudaSiftH.cu featureUploadH.cu cudaSiftD.cu 
   )

add_custom_target(PySiftCuda ALL
   DEPENDS ${OUTDIR}/${LIBNAME}
   )

install(FILES ${OUTDIR}/${LIBNAME} DESTINATION ${COGX_ROOT}/output/python)
