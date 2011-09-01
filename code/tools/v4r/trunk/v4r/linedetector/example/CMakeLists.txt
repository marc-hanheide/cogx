
include_directories(../)
set(LINEDETECTOR_CPP main_linedetector.cpp)
add_executable(linedetector ${LINEDETECTOR_CPP})
target_link_libraries(linedetector ${OPENCV_LIBRARIES}  v4rlinedetector v4rcontour)
