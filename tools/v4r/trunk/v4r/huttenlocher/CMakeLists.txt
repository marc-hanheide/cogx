SET(V4R_HUTTENLOCHER 1 CACHE BOOL "Add description here." )
IF(V4R_HUTTENLOCHER)

project ( nao2010 )

set(HUTTENLOCHER_CPP hl.cpp)
set(HUTTENLOCHER_H hl.h)
add_library(v4rhl SHARED ${HUTTENLOCHER_CPP}  ${HUTTENLOCHER_H})
target_link_libraries(v4rhl ${OPENCV_LIBRARIES})

add_subdirectory(example)

set(HUTTENLOCHER_DIR ${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(mex)

ENDIF(V4R_HUTTENLOCHER)
