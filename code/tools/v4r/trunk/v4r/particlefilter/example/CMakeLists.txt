add_executable(pf main_mcl.cpp)
target_link_libraries(pf ${OPENCV_LIBRARIES} ${BOOST_LIBRARIES} v4rpf v4rutils v4rgstatistic)

add_executable(pf_pendulum main_pendulum.cpp)
target_link_libraries(pf_pendulum ${OPENCV_LIBRARIES} ${BOOST_LIBRARIES} v4rpf v4rutils akshm v4rgstatistic)

add_executable(distributions main_distributions.cpp)
target_link_libraries(distributions ${OPENCV_LIBRARIES} ${BOOST_LIBRARIES} v4rpf v4rutils v4rgstatistic)

