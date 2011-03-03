PROJECT(SDTomGineWraper)

include_directories(.)

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
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

