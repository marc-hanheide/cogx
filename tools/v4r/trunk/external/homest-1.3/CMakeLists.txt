SET(V4R_HOMEST 0 CACHE BOOL "Lourakis' homography estimation!" )
IF(V4R_HOMEST)

PROJECT(homest)
SET(SOURCE_CPP
  buckets.c
  calc_2Dhomog_coeffs.c
  homest.c
  linalg.c
  lqs.c
  norm.c
)

SET(SOURCE_H
  compiler.h
  homest.h
  lqs.h
  maplefuncs.h
  util.h
)

add_library(homest SHARED ${SOURCE_CPP})
target_link_libraries(homest levmar)

add_subdirectory(demo)

ENDIF(V4R_HOMEST)

