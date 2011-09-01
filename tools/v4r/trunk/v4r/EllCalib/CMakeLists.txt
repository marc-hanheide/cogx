SET(V4R_ELLCALIB 1 CACHE BOOL "Add description here." )
IF(V4R_ELLCALIB)

# elldetect -----------------------------------------
PROJECT(EllCalib)
SET(SOURCES
  EllPatternDetector.cc
  GTPatternDetector.cc
  EllCalibrate.cc
  DetectGTPose.cc
  arGTPatternDetector.cc
  arDetectGTPose.cc
)

add_library(EllCalib SHARED ${SOURCES})
target_link_libraries(EllCalib ${OPENCV_LIBRARIES} elldetect v4rmarker)

add_subdirectory(example)

ENDIF(V4R_ELLCALIB)

