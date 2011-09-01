SET(APPS_SHM2TOMGINE 1 CACHE BOOL "Shared memory interface for tomgine" )
IF(APPS_SHM2TOMGINE)

add_executable(shm2tomegine main_shm2tomegine.cpp)
target_link_libraries(shm2tomegine  akshm ${Boost_LIBRARIES} ${OPENCV_LIBRARIES} v4rtomgine v4rglwindow ftgl freetype)

add_executable(shm2tomegine_write main_shm2tomegine_write.cpp)
target_link_libraries(shm2tomegine_write  akshm ${Boost_LIBRARIES})

ENDIF(APPS_SHM2TOMGINE)
