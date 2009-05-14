PROJECT(PlaymateBindingMotiveGenerator)

# Set the sources.
SET(SOURCES PlaymateBindingMotiveGenerator.cpp)
SET(HEADERS PlaymateBindingMotiveGenerator.hpp)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

# Add libraries to link against.
TARGET_LINK_LIBRARIES(${PROJECT_NAME} BindingMotiveGenerator)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
#INSTALL(FILES ${HEADERS} DESTINATION include/motivation/components/PlaymateBindingMotiveGenerator/)
