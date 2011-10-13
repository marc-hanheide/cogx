SET(V4R_PCL_ADD_ONS_2 0 CACHE BOOL "PCL functions and utils with specific applications: Version 2" )
IF(V4R_PCL_ADD_ONS_2)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(./utils)
include_directories(./functions)
add_subdirectory(./utils)
add_subdirectory(./functions)


PROJECT(PlanePopout2)
SET(SOURCE_CPP
  PlanePopout.cc
)

SET(SOURCE_H
  PlanePopout.hh
)

add_library(v4rplanepopout2 SHARED ${SOURCE_CPP})
target_link_libraries(v4rplanepopout2 ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} v4rpclfunctions2)

#add_subdirectory(example)

add_library(v4rmodelfitter2 SHARED ModelFitter.cpp)
target_link_libraries(v4rmodelfitter2 ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} v4rpclfunctions2)


ENDIF(V4R_PCL_ADD_ONS_2)

