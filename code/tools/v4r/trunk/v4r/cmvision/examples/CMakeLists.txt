
add_executable(cmvision main_cmvision.cpp)
target_link_libraries(cmvision ${OPENCV_LIBRARIES}  v4rcmvision v4rcolors)
