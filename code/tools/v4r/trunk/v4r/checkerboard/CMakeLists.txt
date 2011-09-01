SET(V4R_CHECKERBOARD 1 CACHE BOOL "Add description here." )
IF(V4R_CHECKERBOARD)

set(SOURCE_CPP 
 checkerboard.cpp
)

set(SOURCE_H 
 checkerboard.h

)
add_library(v4rcheckerboard SHARED ${SOURCE_H} ${SOURCE_CPP})

add_subdirectory(example)

ENDIF(V4R_CHECKERBOARD)
