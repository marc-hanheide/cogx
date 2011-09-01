SET(AK_SHM 1 CACHE BOOL "shared memory abstraction" )
IF(AK_SHM)

include_directories(${V4R_DIR})

set(SHM_CPP 
  shmmanager.cpp
  shm.cpp )
set(SHM_HPP 
  constants.hpp 
  shm.hpp
  shmarray.hpp 
  shmheader.hpp
  shmlist.hpp   
  shmmanager.hpp 
  shmmap.hpp  
  shmvar.hpp 
  shmvector.hpp 
  shmcvmat.hpp 
  shmimage.hpp  )


add_library(akshm SHARED ${SHM_CPP}  ${SHM_HPP})

add_subdirectory(tools)
add_subdirectory(examples)

set(SHM_DIR ${CMAKE_CURRENT_SOURCE_DIR})


INSTALL(FILES ${SHM_HPP} 
 DESTINATION /usr/include/ak/shm)
INSTALL(TARGETS akshm
 LIBRARY DESTINATION /usr/lib/
)

add_subdirectory(mex)

ENDIF(AK_SHM)
