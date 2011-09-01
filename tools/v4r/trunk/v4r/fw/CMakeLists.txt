SET(V4R_FW 1 CACHE BOOL "Add description here." )
IF(V4R_FW)

PROJECT(ARMarker)

set(FW_CPP 
 fw.cpp 
)
set(FW_H 
 fw.h
)
set(FW_LIBRARIES  dc1394 raw1394)

add_library(v4rfw SHARED ${FW_H} ${FW_CPP})
add_subdirectory(examples)

ENDIF(V4R_FW)
