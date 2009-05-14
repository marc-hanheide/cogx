# Start feature learning project.
PROJECT(SystemUtils)

SET(SOURCES 
  ETimer.cpp
  Common.cpp
  Report.cpp
)

SET(HEADERS
  ETimer.h 
  Common.h
  Report.h
)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} FrameworkIDL)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/vision/components/common/SystemUtils)
