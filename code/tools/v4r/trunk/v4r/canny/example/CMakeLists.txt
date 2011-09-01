
add_executable(canny main_canny.cpp)
target_link_libraries(canny ${OPENCV_LIBRARIES}  v4rcanny)

