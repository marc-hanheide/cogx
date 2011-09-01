SET(V4R_PGEOMETRY 1 CACHE BOOL "Simple geometry stuff (lines, homography,...)." )
IF(V4R_PGEOMETRY)

INCLUDE_DIRECTORIES(.)

set(SOURCE_CPP 
 Pose.cc
 nrutil.c
 fitcircle.c
 SPlane3D.cc
)

set(SOURCE_H 
 PHomography.hh
 SLine2D.hh
 SCircle2D.hh
 Pose.hh
 nrutil.h
 fitcircle.h
 SPlane3D.hh
)

add_library(PGeometry SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(PGeometry ${OPENCV_LIBRARIES})

ENDIF(V4R_PGEOMETRY)
