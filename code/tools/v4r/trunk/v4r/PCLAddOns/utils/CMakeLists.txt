#PROJECT(PCLUtils)
SET(SOURCE_CPP
  PCLUtils.cpp
)

SET(SOURCE_H
  PCLUtils.h
)

add_library(PCLUtils SHARED ${SOURCE_CPP})
target_link_libraries(PCLUtils ${OPENCV_LIBRARIES} ${PCL_LIBRARIES})

