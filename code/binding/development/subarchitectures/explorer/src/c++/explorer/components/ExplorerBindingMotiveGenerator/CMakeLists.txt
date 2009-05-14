PROJECT(ExplorerBindingMotiveGenerator)

# Set the sources.
SET(SOURCES ExplorerBindingMotiveGenerator.cpp)
SET(HEADERS ExplorerBindingMotiveGenerator.hpp)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

# Add libraries to link against.
TARGET_LINK_LIBRARIES(${PROJECT_NAME} BindingMotiveGenerator)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
