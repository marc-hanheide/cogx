
add_executable(testCalib testCalib.cc)
target_link_libraries(testCalib ${OPENCV_LIBRARIES} elldetect EllCalib)

add_executable(testPatternDetector testPatternDetector.cc)
target_link_libraries(testPatternDetector ${OPENCV_LIBRARIES} elldetect EllCalib)

add_executable(ellcalib-live ellcalib-live.cc)
target_link_libraries(ellcalib-live ${OPENCV_LIBRARIES} elldetect EllCalib)

add_executable(ellcalib-file ellcalib-file.cc)
target_link_libraries(ellcalib-file ${OPENCV_LIBRARIES} elldetect EllCalib)

add_executable(testGTPoseDetector testGTPoseDetector.cc)
target_link_libraries(testGTPoseDetector ${OPENCV_LIBRARIES} elldetect EllCalib)

add_executable(testARGTPoseDetector testARGTPoseDetector.cc)
target_link_libraries(testARGTPoseDetector ${OPENCV_LIBRARIES} elldetect EllCalib)

