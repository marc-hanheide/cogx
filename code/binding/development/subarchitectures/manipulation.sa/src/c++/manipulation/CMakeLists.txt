PROJECT(Manipulation)

# Set the sources.
SET(SOURCES ManipulationGoals.cpp)
SET(HEADERS ManipulationGoals.h)

# Add the Manipulation library to the build targets.
ADD_LIBRARY(Manipulation SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(Manipulation ${OMNIORB4_LIBRARIES})
TARGET_LINK_LIBRARIES(Manipulation CASTIDL ManipulationIDL)

# Add libraries to link against.

INSTALL(TARGETS Manipulation LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/manipulation/)
