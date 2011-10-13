#PROJECT(PCLFunctions)
SET(SOURCE_CPP
  CCLabeling.cc
  PCLFunctions.cpp
)

SET(SOURCE_H
  CCLabeling.hh
  PCLFunctions.h
)

add_library(v4rpclfunctions SHARED ${SOURCE_CPP})
target_link_libraries(v4rpclfunctions ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} v4rpclutils)

