find_path(DLIB_INCLUDE_DIR dlib/bayes_utils.h /home/goebelbe/src/dlib-17.31)

include_directories(.)
include_directories(${DLIB_INCLUDE_DIR})

set(bnet_source
  main.cpp)

add_executable(bnet ${bnet_source})
install(TARGETS bnet DESTINATION bin)
