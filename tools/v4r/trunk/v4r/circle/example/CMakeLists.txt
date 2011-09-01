add_executable(circle_opencv main_circle_opencv.cpp)
target_link_libraries(circle_opencv ${OPENCV_LIBRARIES} ${BOOST_LIBRARIES} v4rutils)
