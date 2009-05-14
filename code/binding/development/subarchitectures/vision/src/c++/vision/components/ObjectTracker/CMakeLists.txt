# Start feature learning project.
PROJECT(ObjectTracker)

SET(SOURCES
  ObjectTracker.cpp
  Tracker.cpp
  BTComponents/BlobTracker.cpp
  BTComponents/TrackAux.cpp
  BTComponents/TrackerConfig.cpp
  BTComponents/MobileObject.cpp
  BTComponents/ViewTracker.cpp
  BTComponents/ObjView.cpp
  BTComponents/CircleView.cpp
  BTComponents/RectView.cpp
  BTComponents/TrackerAbstract.cpp
  BTComponents/ObjectTypeManage.cpp
)
SET(HEADERS
  ObjectTracker.h
  Tracker.h
  BTComponents/BlobTracker.h
  BTComponents/TrackAux.h
  BTComponents/TrackerConfig.h
  BTComponents/MobileObject.h
  BTComponents/ViewTracker.h
  BTComponents/ObjView.h
  BTComponents/CircleView.h
  BTComponents/RectView.h
  BTComponents/TrackerAbstract.hpp
  BTComponents/ObjectTypeManage.hpp
)

# definitions for Geometry part
ADD_DEFINITIONS(-DMILLIMETER)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} VisionIDL Vision)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} VisionUtils)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} GeomUtils SystemUtils VisualizationUtils)

# Use pkg-config to find opencv libraries...
INCLUDE(UsePkgConfig)
PKGCONFIG("opencv" OCV_INCLUDE_DIR OCV_LINK_DIR OCV_LINK_FLAGS OCV_CFLAGS)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OCV_LINK_FLAGS})

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/vision/components/ObjectTracker)
