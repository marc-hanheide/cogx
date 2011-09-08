SET(V4R_DAISY 1 CACHE BOOL "Add description here." )
IF(V4R_DAISY)

set(DAISY_CPP 
  daisy.cpp   
  interaction.cpp  
  matrix_manipulation.cpp
  general.cpp 
  math.cpp  
  fileio.cpp)
set(DAISY_H 
  daisy.h
  fileio.h  
  general.h  
  interaction.h  
  kutility.def  
  math.h  
  matrix_manipulation.h)

IF(UNIX AND NOT WIN32)
  FIND_PROGRAM(CMAKE_UNAME uname /bin /usr/bin /usr/local/bin )
  IF(CMAKE_UNAME)
    EXEC_PROGRAM(uname ARGS -m OUTPUT_VARIABLE CMAKE_SYSTEM_PROCESSOR)
    SET(CMAKE_SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR} CACHE INTERNAL "processor type (i386 and x86_64)")
    IF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
      ADD_DEFINITIONS(-fPIC)
    ENDIF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  ENDIF(CMAKE_UNAME)
ENDIF(UNIX AND NOT WIN32)

add_library(v4rdaisy SHARED ${DAISY_CPP}  ${DAISY_H})
target_link_libraries(v4rdaisy ${OPENCV_LIBRARIES})
add_subdirectory(example)

set(DAISY_DIR ${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(mex)

ENDIF(V4R_DAISY)
