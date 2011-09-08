SET(V4R_UTILS 1 CACHE BOOL "Add description here." )
IF(V4R_UTILS)

set(UTILS_CPP 
 distributions.cpp 
 string.cpp 
 filesystemhdl.cpp
 timehdl.cpp
)
set(UTILS_H
 distributions.h 
 string.h
 filesystemhdl.h
 timehdl.h
)


IF(NOT DEFINED V4R_COMPILE_STATIC)
add_library(v4rutils SHARED ${UTILS_CPP} ${UTILS_H})
ELSE()
add_library(v4rutils STATIC ${UTILS_CPP} ${UTILS_H})
ENDIF()

set(Boost_USE_MULTITHREADED ON)
FIND_PACKAGE (Boost 1.38.0 REQUIRED COMPONENTS date_time thread program_options serialization regex filesystem signals system)
target_link_libraries(v4rutils ${Boost_LIBRARIES})

INSTALL(FILES ${UTILS_H} 
 DESTINATION include/v4r/utils)
INSTALL(TARGETS v4rutils
 LIBRARY DESTINATION lib/
)

ENDIF(V4R_UTILS)

