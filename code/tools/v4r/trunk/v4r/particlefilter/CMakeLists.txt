SET(V4R_PARTICLEFILTER 1 CACHE BOOL "Add description here." )
IF(V4R_PARTICLEFILTER)

set(PF_CPP 
 pfilter.cpp 
 particle.cpp 
 particle_set.cpp 
 particle_storage.cpp 
 linear.cpp 
 linear_set.cpp 
 random.cpp 
 random_set.cpp 
  pendulum.cpp pendulum_fnc.cpp pendulum_set.cpp
)

set(PF_H 
 pfilter.h
 particle.h
 linear.h
 random.h
 pendulum.h 
)


add_library(v4rpfrobot STATIC robot2d.h robot2d.cpp robot2d_set.cpp)
add_library(v4rpfpendulum STATIC pendulum.cpp pendulum_fnc.cpp pendulum_set.cpp)
add_library(v4rpf SHARED ${PF_CPP} ${PF_H})
target_link_libraries(v4rpf v4rpfrobot )
target_link_libraries(v4rpf v4rutils ${OPENCV_LIBRARIES})

add_subdirectory(example)

ENDIF(V4R_PARTICLEFILTER)
