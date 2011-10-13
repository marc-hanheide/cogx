#PROJECT(PCLFunctions)
SET(SOURCE_CPP
  CCLabeling.cc
  PCLFunctions.cpp
)

SET(SOURCE_H
  CCLabeling.hh
  PCLFunctions.h
)

add_library(PCLFunctions SHARED ${SOURCE_CPP})
target_link_libraries(PCLFunctions ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} PCLUtils)

