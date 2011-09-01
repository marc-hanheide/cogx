SET(AK_SHM_CV_EXAMPLE 1 CACHE BOOL "examples to show the use of opencv data with akshm" )
IF(AK_SHM_CV_EXAMPLE )

add_executable(example_shmimage shmimage.cpp )
target_link_libraries(example_shmimage akshm ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(example_shmcvmat shmcvmat.cpp )
target_link_libraries(example_shmcvmat akshm ${Boost_LIBRARIES} ${OpenCV_LIBS})

ENDIF(AK_SHM_CV_EXAMPLE )

add_executable(example_shmvar shmvar.cpp )
target_link_libraries(example_shmvar akshm ${Boost_LIBRARIES})
