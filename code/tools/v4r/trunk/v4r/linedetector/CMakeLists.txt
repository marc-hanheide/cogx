SET(V4R_LINEDETECTOR 1 CACHE BOOL "Add description here." )
IF(V4R_LINEDETECTOR)

EXECUTE_PROCESS( COMMAND hostname OUTPUT_VARIABLE HOSTNAME)

IF ( ${HOSTNAME} STREQUAL "rudi"  )

set(LINEDETECTOR_H linedetector.h)
set(LINEDETECTOR_CPP linedetector.cpp)
add_library(v4rlinedetector SHARED ${LINEDETECTOR_H} ${LINEDETECTOR_CPP})
add_subdirectory(example)
ELSE()
MESSAGE( STATUS "Skipping linedetector, line detector is done on host: rudi")
ENDIF()

ENDIF(V4R_LINEDETECTOR)
