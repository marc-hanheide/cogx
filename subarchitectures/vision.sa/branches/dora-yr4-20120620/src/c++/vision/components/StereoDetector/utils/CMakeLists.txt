PROJECT(SDUtils)

include_directories(.)

# Set the sources.
SET(HEADERS
  CDataFile.h
  Color.hh
  ColorHistogram.h
  Config.hh
  ConfigFile.hh
  Draw.hh
  fitcircle.h
  fitellipse.h
  Histogram.h
  Namespace.hh
  OpenCvImgSeqVideo.hh
  OpenCvLiveVideo.hh
  Video.hh
)

SET(SOURCES 
  CDataFile.cpp
  Color.cc
  ColorHistogram.cpp
  Config.cc
  ConfigFile.cc
  Draw.cc
  Histogram.cpp
)


ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

