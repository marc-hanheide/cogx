SET(V4R_FIGURE 1 CACHE BOOL "2D Plot lib based on OpenCV" )

IF(V4R_FIGURE)

set(SRC_FILES 
 figure.cpp 
)

set(H_FILES 
 figure.h 
)

add_library(v4rfigure STATIC ${H_FILES} ${SRC_FILES})

add_subdirectory(examples)
ENDIF(V4R_FIGURE)
