
add_executable(recogniser recogniser.cc)
target_link_libraries(recogniser ${OPENCV_LIBRARIES} CModelRecogniser)

add_executable(recogniserThread recogniserThread.cc)
target_link_libraries(recogniserThread ${OPENCV_LIBRARIES} CModelRecogniser)

add_executable(itmosPlanes itmosPlanes.cc)
target_link_libraries(itmosPlanes ${OPENCV_LIBRARIES} CModelRecogniser)

