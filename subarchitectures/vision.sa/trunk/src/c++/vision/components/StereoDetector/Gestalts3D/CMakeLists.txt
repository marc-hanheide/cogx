PROJECT(SDGestalts3D)

include_directories(.)

# Set the header and sources.
SET(HEADERS
  Closure3D.h
  Collinearity3D.h
  Corner3D.h
  Ellipse3D.h
  Flap3D.h
  Gestalt3D.h
  Line3D.h
  LJunction3D.h
  Patch3D.h
  Rectangle3D.h
  Segment3D.h
)

SET(SOURCES 
  Closure3D.cpp
  Collinearity3D.cpp
  Corner3D.cpp
  Ellipse3D.cpp
  Flap3D.cpp
  Gestalt3D.cpp
  Line3D.cpp
  LJunction3D.cpp
  Patch3D.cpp
  Rectangle3D.cpp
  Segment3D.cpp
)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

