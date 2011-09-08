SET(V4R_EDGELINKER 1 CACHE BOOL "Add description here." )
IF(V4R_EDGELINKER)

add_library(v4rcontour SHARED contour.cpp contour.h)
add_library(v4redgelinker SHARED edgelinker.cpp edgelinker.h)
target_link_libraries(v4redgelinker ${OPENCV_LIBRARIES})
target_link_libraries(v4rcontour ${OPENCV_LIBRARIES})

add_subdirectory(example)

ENDIF(V4R_EDGELINKER)
