SET(V4R_LPSEGMENT 0 CACHE BOOL "Log-polar segmenter" )
IF(V4R_LPSEGMENT)

PROJECT(CModelRecogniser)
SET(SOURCE_CPP
  LogPolar.cc
  LPSegment.cc
  RGBColourProb.cc
  RGBHistogram.cc
  UVColourProb.cc
  UVHistogram.cc
)

SET(SOURCE_H
  LogPolar.hh
  LPNode.hh
  RGBColourProb.hh
  UVColourProb.hh
  LPDefinitions.hh
  LPSegment.hh
  RGBHistogram.hh
  UVHistogram.hh
)

add_library(LPSegment SHARED ${SOURCE_CPP})
target_link_libraries(LPSegment ${OPENCV_LIBRARIES} CEdge PMath)

add_subdirectory(example)

ENDIF(V4R_LPSEGMENT)

