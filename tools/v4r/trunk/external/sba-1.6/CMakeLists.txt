SET(V4R_SBA 0 CACHE BOOL "Lourakis' Sparse Bundle Adjustment!" )
IF(V4R_SBA)

PROJECT(sba)
SET(SOURCE_CPP
  sba_levmar.c
  sba_levmar_wrap.c
  sba_lapack.c
  sba_crsm.c
  sba_chkjac.c
  SBAWrapper.cc
)

SET(SOURCE_H
  sba.h
  sba_chkjac.h
  compiler.h
  SBAWrapper.hh
)

add_library(sba SHARED ${SOURCE_CPP})
target_link_libraries(sba levmar PGeometry PMath ${OPENCV_LIBRARIES})

add_subdirectory(demo)

ENDIF(V4R_SBA)

