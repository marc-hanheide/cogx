add_executable(camerageometry main_camera.cpp)
target_link_libraries(camerageometry ${OPENCV_LIBRARIES} v4rgeometry v4rcamerageometry)

