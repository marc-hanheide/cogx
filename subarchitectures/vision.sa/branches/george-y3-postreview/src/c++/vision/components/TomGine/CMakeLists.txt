include_directories(.)

add_definitions(-DLINUX)

include_directories(GLWindow)
include_directories(TomGine)
include_directories(TomGineWraper)

add_subdirectory(GLWindow)
add_subdirectory(TomGine)
add_subdirectory(TomGineWraper)

