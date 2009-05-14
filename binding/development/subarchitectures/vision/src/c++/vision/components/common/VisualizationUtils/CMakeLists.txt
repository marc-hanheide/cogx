# Start feature learning project.
PROJECT(VisualizationUtils)

SET(SOURCES 
  Display.cpp
  Visualization.cpp
)

SET(HEADERS
  ColorDefs.h
  Display.h
  Visualization.h
)

# definitions for Geometry part
ADD_DEFINITIONS(-DMILLIMETER)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

#TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LIBRARIES})
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} VisionIDL Vision VisionUtils)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} VisionIDL)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} SystemUtils GeomUtils)


# Use pkg-config to find opencv libraries...
INCLUDE(UsePkgConfig)
PKGCONFIG("opencv" OCV_INCLUDE_DIR OCV_LINK_DIR OCV_LINK_FLAGS OCV_CFLAGS)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OCV_LINK_FLAGS})

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/vision/components/common/Visualization)
