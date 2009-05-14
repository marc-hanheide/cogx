# Start feature learning project.
PROJECT(GeomUtils)

SET(SOURCES 
  Vector2D.cpp
  Vector3D.cpp
  Model3D.cpp
  CameraProj.cpp
  Polygon3D.cpp
  Polygon2D.cpp
  Polygon.cpp
  Plane.cpp
  Ray.cpp
  Intersect.cpp
  Segment2D.cpp
  GeomUtils.cpp
)

SET(HEADERS
  GeomMath.h
  Vector2D.h
  Vector3D.h
  Model3D.h
  CameraProj.h
  Polygon3D.h
  Polygon2D.h
  Polygon.h
  Plane.h
  Ray.h
  Intersect.h
  Segment2D.h
  GeomUtils.hpp
)

# definitions for Geometry part
ADD_DEFINITIONS(-DMILLIMETER)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} SystemUtils)

# Use pkg-config to find opencv libraries...
INCLUDE(UsePkgConfig)
PKGCONFIG("opencv" OCV_INCLUDE_DIR OCV_LINK_DIR OCV_LINK_FLAGS OCV_CFLAGS)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OCV_LINK_FLAGS})

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/vision/components/common/GeomUtils)
