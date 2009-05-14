PROJECT(BindingMotiveGenerator)

# Set the sources.
SET(SOURCES BindingMotiveTemplates.cpp BindingMotiveGenerator.cpp)
SET(HEADERS BindingMotiveTemplates.hpp BindingMotiveGenerator.hpp)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

# Add libraries to link against.
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CASTArchitecture)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} BindingIDL BindingSubarch)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PlanningSubarchUtil)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} MotivationData)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/motivation/components/BindingMotiveGenerator/)
