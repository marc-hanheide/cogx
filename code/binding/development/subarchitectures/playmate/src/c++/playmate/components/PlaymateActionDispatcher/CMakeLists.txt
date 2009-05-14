PROJECT(PlaymateActionDispatcher)

# Set the sources.
SET(SOURCES PlaymateActionDispatcher.cpp)
SET(HEADERS PlaymateActionDispatcher.hpp)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

# Add libraries to link against.
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ActionDispatcher)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} SpatialIDL)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ManipulationIDL)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} VisionIDL)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
#INSTALL(FILES ${HEADERS} DESTINATION include/action/components/ActionDispatcher/)
