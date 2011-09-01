add_executable(figure2d_example main_figure2d.cpp)
target_link_libraries(figure2d_example ${OPENCV_LIBRARIES} ${BOOST_LIBRARIES} v4rfigure)
