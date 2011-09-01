SET(V4R_COLORS 1 CACHE BOOL "Add description here." )
IF(V4R_COLORS)

IF(NOT DEFINED V4R_COMPILE_STATIC)
add_library(v4rcolors SHARED colors.cpp colors.h)
ELSE()
add_library(v4rcolors STATIC colors.cpp colors.h)
ENDIF()

add_subdirectory(examples)

ENDIF(V4R_COLORS)

