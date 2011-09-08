SET(V4R_FW 1 CACHE BOOL "Add description here." )
IF(V4R_FW)

PROJECT(ARMarker)

set(FW_CPP 
 fw.cpp 
)
set(FW_H 
 fw.h
)

IF (NOT APPLE)
  set(FW_LIBRARIES  dc1394 raw1394)
ELSE (NOT APPLE)
  set(FW_LIBRARIES  dc1394) 
ENDIF (NOT APPLE)


add_library(v4rfw SHARED ${FW_H} ${FW_CPP})
target_link_libraries(v4rfw ${OPENCV_LIBRARIES} ${FW_LIBRARIES})

add_subdirectory(examples)

ENDIF(V4R_FW)
