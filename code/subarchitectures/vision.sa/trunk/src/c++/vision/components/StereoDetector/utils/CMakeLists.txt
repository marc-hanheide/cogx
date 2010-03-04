PROJECT(SDUtils)

include_directories(.)

# Set the sources.
SET(HEADERS
	Array.hh
#	BufferVideo.hh
	CDataFile.h
	Color.hh
	Config.hh
	ConfigFile.hh
	Draw.hh
	Except.hh
	Namespace.hh
	OpenCvImgSeqVideo.hh
	OpenCvLiveVideo.hh
	Video.hh
	Vector2.hh
)

SET(SOURCES 
	Array.ic
#	BufferVideo.cc
	CDataFile.cpp
	Color.cc
	Config.cc
	ConfigFile.cc
	Draw.cc
#	Draw_small.cc
	Except.cc
#	OpenCvImgSeqVideo.cc
#	OpenCvLiveVideo.cc
	Vector2.ic
	Vector2.cc
)


ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

