SET(V4R_CMVISION 1 CACHE BOOL "Add description here." )
IF(V4R_CMVISION)

add_subdirectory(CMVision-1.2.0)
include_directories(./CMVision-1.2.0/include )

add_library(v4rcmvision SHARED segmentation.cpp segmentation.h )
target_link_libraries(v4rcmvision cmvision1.2)
add_subdirectory(examples)

ENDIF(V4R_CMVISION)
