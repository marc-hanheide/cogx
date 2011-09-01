SET(V4R_LEVMAR 0 CACHE BOOL "Lourakis' Levenberg-Marquardt Algorithm" )
IF(V4R_LEVMAR)

PROJECT(levmar)
SET(SOURCE_CPP
  lm.c Axb.c misc.c lmlec.c lmbc.c lmblec.c lmbleic.c
)

SET(SOURCE_H
  levmar.h misc.h compiler.h
)

add_library(levmar SHARED ${SOURCE_CPP})
target_link_libraries(levmar lapack blas)

ENDIF(V4R_LEVMAR)

