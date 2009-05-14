PROJECT(ManipulationWorkingMemory)

SET(SOURCES ManipulationWorkingMemory.cpp)
SET(HEADERS ManipulationWorkingMemory.h)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ManipulationIDL Manipulation Vision VisionIDL)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/manipulation/components/ManipulationWorkingMemory)
