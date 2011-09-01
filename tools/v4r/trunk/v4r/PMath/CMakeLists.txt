SET(V4R_PMATH 1 CACHE BOOL "Add description here." )
IF(V4R_PMATH)

set(SOURCE_CPP 
  PMath.cc
)

set(SOURCE_H 
 PMath.hh
 PVector.hh
 PMatrix.hh
)

add_library(PMath SHARED ${SOURCE_H} ${SOURCE_CPP})

ENDIF(V4R_PMATH)
