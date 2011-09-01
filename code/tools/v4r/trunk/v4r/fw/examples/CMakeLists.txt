
include_directories(./)
set(CMAKE_CXX_FLAGS "-Wall -g -D __STDC_FORMAT_MACROS")


add_executable(fw_bayer main_fw_bayer.cpp)
target_link_libraries(fw_bayer v4rfw ${FW_LIBRARIES} ${OPENCV_LIBRARIES})

add_executable(fw_fast main_fw_fast.cpp)
target_link_libraries(fw_fast v4rfw ${FW_LIBRARIES} ${OPENCV_LIBRARIES})