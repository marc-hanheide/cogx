PROJECT(Vs3MxCvTools)

include_directories(.)

# Set the sources.
SET(SOURCES 
	mxCameraModel.cpp
	mxDrawTools.cpp)
SET(HEADERS 
	marker.h
	mxCameraModel.h
	mxDrawTools.h)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

