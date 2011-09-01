SET(V4R_GLWINDOW 1 CACHE BOOL "Window creation for OpenGL applications (for Windows and Linux)." )
IF(V4R_GLWINDOW)

add_definitions(-DLINUX)

SET(SOURCES GLXWindow.cpp GLXWindowGetEvent.cpp)
add_library(v4rglwindow SHARED ${SOURCES})
target_link_libraries(v4rglwindow GL GLU rt)

add_subdirectory(example)

ENDIF(V4R_GLWINDOW)
