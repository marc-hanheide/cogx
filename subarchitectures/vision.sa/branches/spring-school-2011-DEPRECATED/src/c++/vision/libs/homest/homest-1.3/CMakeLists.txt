# Based on the LevMar CMakeLists.txt
#
PROJECT(HOMEST)
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

# compiler flags
#ADD_DEFINITIONS(-DLINSOLVERS_RETAIN_MEMORY) # do not free memory between linear solvers calls
#REMOVE_DEFINITIONS(-DLINSOLVERS_RETAIN_MEMORY) # free memory between calls
ADD_DEFINITIONS(-fPIC)

# f2c is sometimes equivalent to libF77 & libI77; in that case, set HAVE_F2C to 0
SET(HAVE_F2C 1 CACHE BOOL "Do we have f2c or F77/I77?" )

# the directory where the lapack/blas/f2c libraries reside
SET(LAPACKBLAS_DIR /usr/lib CACHE PATH "Path to lapack/blas libraries")

# actual names for the lapack/blas/f2c libraries
SET(LAPACK_LIB lapack CACHE STRING "The name of the lapack library")
SET(BLAS_LIB blas CACHE STRING "The name of the blas library")
IF(HAVE_F2C)
   # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   # CogX: if you get the following error:
   #     libf2c.so: undefined reference to `MAIN__'
   # Try to set F2C_LIB=f2c.a in ccmake.
   # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  SET(F2C_LIB f2c CACHE STRING "The name of the f2c library (use f2c.a if you get errors)")
ELSE(HAVE_F2C)
  SET(F77_LIB libF77 CACHE STRING "The name of the F77 library")
  SET(I77_LIB libI77 CACHE STRING "The name of the I77 library")
ENDIF(HAVE_F2C)

########################## NO CHANGES BEYOND THIS POINT ##########################

INCLUDE_DIRECTORIES(.)
INCLUDE_DIRECTORIES(../../levmar/levmar-2.5)

# homest library source files
ADD_LIBRARY(homest STATIC
   calc_2Dhomog_coeffs.c homest.c lqs.c linalg.c norm.c buckets.c
   )

# demo program
if (LIBBUILD_LEVMAR_HOMEST_DEMO)
   LINK_DIRECTORIES(${LAPACKBLAS_DIR})
   LINK_DIRECTORIES(.)
   ADD_EXECUTABLE(homest_demo homest_demo.c)
   # libraries the demo depends on
   IF(HAVE_F2C)
      TARGET_LINK_LIBRARIES(homest_demo homest levmar ${LAPACK_LIB} ${BLAS_LIB} ${F2C_LIB})
   ELSE(HAVE_F2C)
      TARGET_LINK_LIBRARIES(homest_demo homest levmar ${LAPACK_LIB} ${BLAS_LIB} ${F77_LIB} ${I77_LIB})
   ENDIF(HAVE_F2C)

   # make sure that the library is built before the demo
   ADD_DEPENDENCIES(homest_demo homest levmar)
endif (LIBBUILD_LEVMAR_HOMEST_DEMO)

#SUBDIRS(matlab)

#ADD_TEST(levmar_tst lmdemo)

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# CogX: installation
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if(COGX_ROOT)
   install(TARGETS homest DESTINATION ${COGX_ROOT}/output/lib)
endif(COGX_ROOT)

