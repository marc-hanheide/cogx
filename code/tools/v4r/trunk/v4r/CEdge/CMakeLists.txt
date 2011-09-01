SET(V4R_CEDGE 1 CACHE BOOL "Add description here." )
IF(V4R_CEDGE)
cmake_minimum_required(VERSION 2.6)

FIND_PACKAGE(OpenCV REQUIRED )


INCLUDE_DIRECTORIES(.)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})


SET(SOURCES
 gammln.c
 betacf.c
 betai.c
 nrutil.c
 SDraw.cc
 Math.cc
 Vector2.cc
 Except.cc
 CEdge.cc
 Segment.cc
 Line.cc
 Arc.cc
 rosin_lines.cc
 fitcircle.c
 FormArcs.cc
 FormLines.cc
 CreateMSLD.cc
 DetectMSLD.cc
)

set(CMAKE_CXX_FLAGS "-Wall -g") #debug

ADD_LIBRARY(CEdge SHARED ${SOURCES})
INSTALL(TARGETS CEdge LIBRARY DESTINATION lib)

add_subdirectory(example)

ENDIF(V4R_CEDGE)
