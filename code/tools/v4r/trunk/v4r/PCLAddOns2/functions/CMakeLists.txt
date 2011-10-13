#PROJECT(PCLFunctions)
SET(SOURCE_CPP
  CCLabeling.cc
  PCLFunctions.cpp
)

SET(SOURCE_H
  CCLabeling.hh
  PCLFunctions.h
)

add_library(v4rpclfunctions2 SHARED ${SOURCE_CPP})
target_link_libraries(v4rpclfunctions2 ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} v4rpclutils2)

