# Start simple action generation project.
PROJECT(SimpleActionGenerator)

SET(SOURCES SimpleActionGenerator.cpp)

SET(HEADERS SimpleActionGenerator.hpp)

# definitions for Geometry part
ADD_DEFINITIONS(-DCENTIMETER)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} PlanningSubarchAbstr)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PlanningSubarchUtil)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PlanningData)


INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/planning/components/SimpleActionGenerator)
