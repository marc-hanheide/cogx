#PROJECT(PCLUtils)
SET(SOURCE_CPP
  PCLUtils.cpp
)

SET(SOURCE_H
  PCLUtils.h
)

add_library(v4rpclutils SHARED ${SOURCE_CPP})
target_link_libraries(v4rpclutils ${OPENCV_LIBRARIES} ${PCL_LIBRARIES})

