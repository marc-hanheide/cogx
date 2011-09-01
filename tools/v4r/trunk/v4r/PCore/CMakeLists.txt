SET(V4R_PCORE 1 CACHE BOOL "Some stuff..." )
IF(V4R_PCORE)

INCLUDE_DIRECTORIES(.)

set(SOURCE_CPP 
)

set(SOURCE_H
  toString.hpp
  Point3fCol.hh
  ImgUtils.hh
)

#add_library(PCore SHARED ${SOURCE_H} ${SOURCE_CPP})

ENDIF(V4R_PCORE)
