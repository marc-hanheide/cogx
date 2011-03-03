PROJECT(SDObjRepresentation)

include_directories(.)

# Set the sources.
SET(HEADERS
	Link.h
	Node.h
	ObjRep.h
)

SET(SOURCES 
	Link.cpp
	Node.cpp
	ObjRep.cpp
)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

