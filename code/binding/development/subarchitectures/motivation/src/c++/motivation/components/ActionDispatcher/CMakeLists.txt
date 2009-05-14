PROJECT(ActionDispatcher)

# Set the sources.
SET(SOURCES ActionDispatcher.cpp)
SET(HEADERS ActionDispatcher.hpp)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

# Add libraries to link against.
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CASTArchitecture)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} BindingSubarch)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PlanningSubarchUtil)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} MotivationData)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/action/components/ActionDispatcher/)
