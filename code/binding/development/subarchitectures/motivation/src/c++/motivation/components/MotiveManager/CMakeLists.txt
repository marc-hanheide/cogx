PROJECT(MotiveManager)

# Set the sources.
SET(SOURCES MotiveManager.cpp)
SET(HEADERS MotiveManager.hpp AbstractTranslators.hpp)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

# Add libraries to link against.
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CASTArchitecture)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PlanningSubarchUtil)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} MotivationData)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
#INSTALL(FILES ${HEADERS} DESTINATION include/motivation/components/MotiveManager/)
