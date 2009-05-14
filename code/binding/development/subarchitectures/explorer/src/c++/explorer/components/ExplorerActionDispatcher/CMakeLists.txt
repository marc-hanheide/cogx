PROJECT(ExplorerActionDispatcher)

# Set the sources.
SET(SOURCES ExplorerActionDispatcher.cpp)
SET(HEADERS ExplorerActionDispatcher.hpp)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

# Add libraries to link against.
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ActionDispatcher)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} NavIDL)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
