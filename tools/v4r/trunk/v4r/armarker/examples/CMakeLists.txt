include_directories(${V4R_INCLUDE})

ADD_EXECUTABLE(armarker_sample main.cpp)
target_link_libraries(armarker_sample v4rmarker ${OPENCV_LIBRARIES})


ADD_EXECUTABLE(armarker_pose main_marker_pose.cpp)
target_link_libraries(armarker_pose v4rmarker ${OPENCV_LIBRARIES})

# ADD_EXECUTABLE(armarker_fw main_marker_fw.cpp)
# target_link_libraries(armarker_fw v4rmarker v4rfw ${OPENCV_LIBRARIES})
 