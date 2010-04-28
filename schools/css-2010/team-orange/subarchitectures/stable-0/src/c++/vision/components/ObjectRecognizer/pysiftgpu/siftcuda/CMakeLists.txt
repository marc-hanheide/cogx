cmake_minimum_required(VERSION 2.6)

# ########################################################################### 
# SIFT implementation with pure CUDA
# * Matching works.
# * Extraction works, but is not stable; depends on the image and its size.
# ########################################################################### 

project(PythonSiftCuda)
find_package(PythonLibs)

option(CUDA_LIBCUTIL_HACK "Try this if PythonSiftCuda (siftcuda.so) doesn't build" OFF)
find_file(CUDA_CUT_LIBRARY libcutil.a
   PATHS
      /usr/local/cuda/sdk/lib
      /usr/local/cuda/sdk/C/lib
)

if(CUDA_LIBCUTIL_HACK)
   set(LIBCUTIL ${CUDA_CUT_LIBRARY})
else(CUDA_LIBCUTIL_HACK)
   set(LIBCUTIL "")
endif(CUDA_LIBCUTIL_HACK)

set(OUTDIR ${CMAKE_CURRENT_BINARY_DIR})
set(LIBNAME siftcuda.so)
string(REGEX MATCH "[0-9][.][0-9]$" PYVER a${PYTHON_EXECUTABLE}) 
# message(${PYVER})
add_custom_command(
   OUTPUT ${OUTDIR}/${LIBNAME}
   COMMAND make -f ${CMAKE_CURRENT_SOURCE_DIR}/Makefile 
      SOURCEDIR="${CMAKE_CURRENT_SOURCE_DIR}" DESTDIR="${OUTDIR}"
      PYVERSION="${PYVER}"
      LIBCUTIL="${LIBCUTIL}"
   DEPENDS 
      Makefile
      cudaImage.h cudaSiftD.h cudaSift.h cudaSiftH.h featureUploadH.h tpimage.h tpimageutil.h
      pysiftcuda.cpp 
      cudaImage.cu cudaSiftH.cu featureUploadH.cu cudaSiftD.cu 
   )

add_custom_target(PySiftCuda ALL
   DEPENDS ${OUTDIR}/${LIBNAME}
   )

install(FILES ${OUTDIR}/${LIBNAME} DESTINATION ${COGX_ROOT}/output/python)
