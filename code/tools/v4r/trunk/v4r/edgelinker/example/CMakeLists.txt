
add_executable(contour main_contour.cpp)
target_link_libraries(contour ${OPENCV_LIBRARIES}  v4rcontour)


add_executable(edgelinker main_edgelinker.cpp)
target_link_libraries(edgelinker ${OPENCV_LIBRARIES}  v4redgelinker v4rcanny)

add_executable(contour_stream main_contour_stream.cpp)
target_link_libraries(contour_stream ${OPENCV_LIBRARIES}  v4rcontour ${BOOST_LIBRARIES} v4rutils)
