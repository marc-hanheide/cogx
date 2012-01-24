PROJECT(TomGineWraper)

FIND_PACKAGE( OpenGL REQUIRED )

#include_directories(.)

# Set the sources.
SET(HEADERS
	Pose.hh
	TomGineThread.hh
)

SET(SOURCES 
	Pose.cc
	TomGineThread.cc
)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} TomGine)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

