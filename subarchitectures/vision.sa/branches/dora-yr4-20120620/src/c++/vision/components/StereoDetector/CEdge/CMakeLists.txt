PROJECT(SDCEdge)

INCLUDE_DIRECTORIES(.)

SET(SOURCES
  gammln.c
  betacf.c
  betai.c
  nrutil.c
  SDraw.cc
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

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
