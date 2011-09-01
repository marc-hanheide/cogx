include_directories(../ )
set(HUTTENLOCHER_CPP main_huttenlocher.cpp)
add_executable(huttenlocher ${HUTTENLOCHER_CPP})
target_link_libraries(huttenlocher ${OPENCV_LIBRARIES}  v4rhl)
