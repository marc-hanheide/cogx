PROJECT(PlanningBindingWorkingMemory)

# Set the sources.
SET(SOURCES PlanningBindingWorkingMemory.cpp)
SET(HEADERS PlanningBindingWorkingMemory.hpp)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

# Add libraries to link against.
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LINK_FLAGS} ${OMNIORB4_CFLAGS} ${OMNIORB4_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} FrameworkInterface FrameworkIDL FrameworkCore)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CASTCore CASTArchitecture CASTIDL SubarchitectureWorkingMemory)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PlanningData PlanningSubarch BindingWorkingMemory)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/nav/components/PlanningBindingWorkingMemory)
