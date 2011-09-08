SET(V4R_COLORS 1 CACHE BOOL "Add description here." )
IF(V4R_COLORS)

IF(NOT DEFINED V4R_COMPILE_STATIC)
add_library(v4rcolors SHARED colors.cpp colors.h)
target_link_libraries(v4rcolors ${OPENCV_LIBRARIES})
ELSE()
add_library(v4rcolors STATIC colors.cpp colors.h)
target_link_libraries(v4rcolors ${OPENCV_LIBRARIES})
ENDIF()

add_subdirectory(examples)

ENDIF(V4R_COLORS)

