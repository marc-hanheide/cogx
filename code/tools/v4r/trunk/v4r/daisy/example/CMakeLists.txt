include_directories(./ )
set(DAISY_CORR_CPP main_daisy_corr.cpp)
add_executable(daisycorr ${DAISY_CORR_CPP})
target_link_libraries(daisycorr ${OPENCV_LIBRARIES} ${BOOST_LIBRARIES} v4rdaisy)
