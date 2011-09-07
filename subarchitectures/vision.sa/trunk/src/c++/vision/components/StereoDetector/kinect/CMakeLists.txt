PROJECT(SDKinect)

include_directories(.)

find_package(PCL 1.1 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})

# Set the sources.
SET(HEADERS
  KinectBase.h
  KinectClosures.h
  KinectCollinearities.h
  KinectCore.h
  KinectLines.h
  KinectPatches.h
  KinectRectangles.h
  KinectSegments.h
)

SET(SOURCES 
  KinectBase.cpp
  KinectClosures.cpp
  KinectCollinearities.cpp
  KinectCore.cpp
  KinectLines.cpp
  KinectPatches.cpp
  KinectSegments.cpp
  KinectRectangles.cpp
)

add_cast_component(${PROJECT_NAME} ${SOURCES} ${HEADERS})
link_cast_component(${CAST_COMPONENT_NAME} ${PCL_LIBRARIES})

#ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
#INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

