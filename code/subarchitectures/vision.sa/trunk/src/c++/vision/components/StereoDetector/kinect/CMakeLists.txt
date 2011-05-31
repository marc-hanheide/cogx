PROJECT(SDKinect)

include_directories(.)

find_package(PCL 1.0 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})

# Set the sources.
SET(HEADERS
  KinectBase.h
  KinectCollinearities.h
  KinectCore.h
  KinectLines.h
  KinectPatches.h
  KinectSegments.h
  Patch3D.h
)

SET(SOURCES 
  KinectBase.cpp
  KinectCollinearities.cpp
  KinectCore.cpp
  KinectLines.cpp
  KinectPatches.cpp
  KinectSegments.cpp
  Patch3D.cpp
)

# link_cast_component(${CAST_COMPONENT_NAME} ${PCL_LIBRARIES})


add_cast_component(${PROJECT_NAME} ${SOURCES} ${HEADERS})

#link_cast_component(${CAST_COMPONENT_NAME} SDUtils SDObjRepresentation SDStereo SDMatas SDvs3 SDCEdge SDKinect)
#link_cast_component(${CAST_COMPONENT_NAME} freetype ftgl GLWindow TomGine TomGineWraper)
#link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
#link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)
link_cast_component(${CAST_COMPONENT_NAME} ${PCL_LIBRARIES})



#ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
#INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

