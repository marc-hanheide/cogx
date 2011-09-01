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

add_library(PlanePopout SHARED ${SOURCE_CPP})
target_link_libraries(PlanePopout ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} PCLFunctions)

add_subdirectory(example)

ENDIF(V4R_PCL_ADD_ONS)

