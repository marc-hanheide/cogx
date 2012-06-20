PROJECT(SDGestalts3D)

include_directories(.)

FIND_PACKAGE (OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIR})
link_directories(${OPENCV_LIBRARY_DIRS})

include_directories(${V4R_DIR})

find_package(PCL 1.1 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})

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
  PclCylinder3D.h
  PclLine3D.h
  PclSphere3D.h
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
  PclCylinder3D.cpp
  PclLine3D.cpp
  PclSphere3D.cpp
  Rectangle3D.cpp
  Segment3D.cpp
)


ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
#target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${OPENCV_LIBRARIES})# v4rtomgine)# opencv_core opencv_highgui opencv_imgproc opencv_objdetect opencv_features2d opencv_flann)# v4rtomgine)
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

