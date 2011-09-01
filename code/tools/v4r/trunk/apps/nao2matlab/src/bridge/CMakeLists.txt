add_library(naobridge SHARED naoqi.h  naoqi.cpp)
include_directories(./ ${AL_INCLUDES})


add_executable(naoqi_exe main.cpp)
target_link_libraries(naoqi_exe naobridge naobridge ${AL_LIBRARIES} ${OPENCV_LIBRARIES})
