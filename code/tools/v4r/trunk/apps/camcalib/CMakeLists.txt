SET(APPS_CAMCALIB 1 CACHE BOOL "Add description here." )
IF(APPS_CAMCALIB)

set(Boost_USE_MULTITHREADED ON)
FIND_PACKAGE (Boost 1.38.0 REQUIRED COMPONENTS date_time thread program_options serialization regex filesystem signals)
set (BOOST_LIBRARIES ${Boost_LIBRARIES})

set(CAMCALIB_CPP main_camcalib.cpp)
add_executable(camcalib ${CAMCALIB_CPP})
target_link_libraries(camcalib ${OPENCV_LIBRARIES}  ${BOOST_LIBRARIES} )
include_directories(./ ${AK_INCLUDES})
install (TARGETS camcalib DESTINATION $ENV{HOME}/bin)

ENDIF(APPS_CAMCALIB)
