SET(V4R_PCL_ADD_ONS 0 CACHE BOOL "PCL functions and utils with specific applications." )
IF(V4R_PCL_ADD_ONS)

find_package(PCL 1.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(./utils)
include_directories(./functions)
add_subdirectory(./utils)
add_subdirectory(./functions)


PROJECT(PlanePopout)
SET(SOURCE_CPP
  PlanePopout.cc
)

SET(SOURCE_H
  PlanePopout.hh
)

add_library(v4rplanepopout SHARED ${SOURCE_CPP})
target_link_libraries(v4rplanepopout ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} v4rpclfunctions)

add_subdirectory(example)

add_library(v4rmodelfitter SHARED ModelFitter.cpp)
target_link_libraries(v4rmodelfitter ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} v4rpclfunctions)


ENDIF(V4R_PCL_ADD_ONS)

