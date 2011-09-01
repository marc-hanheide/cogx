add_executable(line3d main_line3D.cpp)
target_link_libraries(line3d ${OPENCV_LIBRARIES} v4rgeometry)

add_executable(gstatistic main_statistic.cpp)
target_link_libraries(gstatistic ${OPENCV_LIBRARIES} v4rgstatistic)
