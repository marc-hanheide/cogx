
add_executable(lpSegment Main.cc)
target_link_libraries(lpSegment ${OPENCV_LIBRARIES} LPSegment)

add_executable(lpSegmentCenter mainCenter.cc)
target_link_libraries(lpSegmentCenter ${OPENCV_LIBRARIES} LPSegment)

