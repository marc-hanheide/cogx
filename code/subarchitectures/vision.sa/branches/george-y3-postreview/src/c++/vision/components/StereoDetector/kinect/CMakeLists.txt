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
#  KinectPclCylinders.h
  KinectPclEdges.h
  KinectPclLines.h
  KinectPclModels.h
  KinectPclSegments.h
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
#  KinectPclCylinders.cpp
  KinectPclEdges.cpp
  KinectPclLines.cpp
  KinectPclModels.cpp
  KinectPclSegments.cpp
  KinectSegments.cpp
  KinectRectangles.cpp
)

add_cast_component(${PROJECT_NAME} ${SOURCES} ${HEADERS})
link_cast_component(${CAST_COMPONENT_NAME} ${PCL_LIBRARIES} v4rTomGine v4rRGBDSegment v4rPCLAddOns)

#ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
#INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

