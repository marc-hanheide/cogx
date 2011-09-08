SET(V4R_GEOMETRY 1 CACHE BOOL "Add description here." )
IF(V4R_GEOMETRY)

set(GEOMETRTY_CPP 
 
 plane.cpp 
 line.cpp 
 blob.cpp
)

set(GEOMETRTY_H 
 
 blob.h 
 structs.h
 plane.h
 line.h
 linebase.h
 line3dbase.h
)

add_library(v4rgeometry SHARED ${GEOMETRTY_H} ${GEOMETRTY_CPP})
target_link_libraries(v4rgeometry ${OPENCV_LIBRARIES})

set(STATISTIC_CPP 
 average.cpp
)

set(STATISTIC_H 
 average.h
)

add_library(v4rgstatistic SHARED ${STATISTIC_H} ${STATISTIC_CPP})

add_subdirectory(camera)
add_subdirectory(examples)

ENDIF(V4R_GEOMETRY)
