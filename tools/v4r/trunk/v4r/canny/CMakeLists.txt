SET(V4R_CANNY 1 CACHE BOOL "Add description here." )
IF(V4R_CANNY)

add_library(v4rcanny SHARED canny.cpp canny.h)
target_link_libraries(v4rcanny ${OPENCV_LIBRARIES})
add_subdirectory(example)

ENDIF(V4R_CANNY)
