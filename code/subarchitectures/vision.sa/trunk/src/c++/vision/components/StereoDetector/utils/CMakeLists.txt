PROJECT(SDUtils)

include_directories(.)

# Set the sources.
SET(HEADERS
	Array.hh
#	BufferVideo.hh
	Color.hh
	Config.hh
	ConfigFile.hh
	Draw.hh
	Except.hh
#	Video.hh
)

SET(SOURCES 
	Array.ic
#	BufferVideo.cc
	Color.cc
	Config.cc
	ConfigFile.cc
	Draw.cc
#	Draw_small.cc
	Except.cc
)


ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

