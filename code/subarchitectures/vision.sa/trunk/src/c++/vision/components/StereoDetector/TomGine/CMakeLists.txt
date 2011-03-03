include_directories(.)

add_definitions(-DLINUX)
#add_definitions(-DUSE_FTGL_FONT)

include_directories(GLWindow)
include_directories(TomGine)

add_subdirectory(GLWindow)
add_subdirectory(TomGine)

