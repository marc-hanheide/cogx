PROJECT(CameraGeometry)

set(GEOMETRTY_CPP 
 camerageometry.cpp
 triangulate.cpp
)

set(GEOMETRTY_H 
 camerageometry.h
 camerageometrybase.h
)


add_library(v4rcamerageometry SHARED ${GEOMETRTY_H} ${GEOMETRTY_CPP})

