
add_executable(colors main_colors.cpp)
target_link_libraries(colors ${OPENCV_LIBRARIES}  v4rcolors v4rcmvision)
