SET(V4R_ELLDETECT 1 CACHE BOOL "Add description here." )
IF(V4R_ELLDETECT)

# elldetect -----------------------------------------
SET(SOURCES
 syFormClosedSegmentGroups.cpp
 codes12bit.cpp
 codes14bit.cpp
 codes20bit.cpp
 multiplatform.cpp
 syHypothesis.cpp
 syLoggerClass.cpp
 SMath.cpp
 syLogger.cpp
 syEdge.cpp
 syMark.cpp
 syEllipse.cpp
 syMarkerDecoder.cpp
 syEllipseDetection.cpp
 sySegment.cpp
 syEllipseHypothesis.cpp
 syStringConversion.cpp
 syEllMethodDualEllipse.cpp
 syVector2.cpp
 syEllMethodSubpixel.cpp
 syVoteImage.cpp
 syExcept.cpp
 EDWrapper.cc
)

ADD_LIBRARY(elldetect SHARED ${SOURCES})

ADD_SUBDIRECTORY(example)

ENDIF(V4R_ELLDETECT)

