PROJECT(SDUtils)

include_directories(.)

# Set the sources.
SET(HEADERS
	CDataFile.h
	Color.hh
	Config.hh
	ConfigFile.hh
	Draw.hh
	fitcircle.h
	fitellipse.h
	Namespace.hh
	OpenCvImgSeqVideo.hh
	OpenCvLiveVideo.hh
	Video.hh
)

SET(SOURCES 
	CDataFile.cpp
	Color.cc
	Config.cc
	ConfigFile.cc
	Draw.cc
)


ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

