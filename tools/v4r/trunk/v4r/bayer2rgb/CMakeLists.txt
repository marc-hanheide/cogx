SET(V4R_BAYER2RGB 1 CACHE BOOL "Add description here." )
IF(V4R_BAYER2RGB)

ADD_EXECUTABLE(bayer2rgb main_bayer2rgb.cpp)
target_link_libraries(bayer2rgb ${OPENCV_LIBRARIES} )
install (TARGETS bayer2rgb DESTINATION $ENV{HOME}/bin)

ENDIF(V4R_BAYER2RGB)
