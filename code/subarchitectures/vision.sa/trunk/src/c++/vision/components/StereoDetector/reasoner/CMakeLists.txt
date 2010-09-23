PROJECT(SDReasoner)

include_directories(.)

# Set the sources.
SET(HEADERS
	Object.h
	Plane.h
	Reasoner.h
)

SET(SOURCES 
	Object.cpp
	Plane.cpp
	Reasoner.cpp
)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

