include_directories(.)
link_directories (/usr/local/lib)
link_directories (/usr/local/cuda/lib)
link_directories (/usr/local/cuda/sdk/lib)
link_directories (${COGX_ROOT}/tools/hardware/stereo/src/c++/components/gpustereo)

add_cast_component(StereoServer StereoServer.cpp StereoCamera.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Stereo Video VideoClient VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} gpustereo)
link_cast_component(${CAST_COMPONENT_NAME} cv highgui cxcore cvaux cudart cutil)


ADD_LIBRARY(StereoClient SHARED StereoClient.cpp)

# Add libraries to link against.
TARGET_LINK_LIBRARIES(StereoClient CASTCore CDL)
TARGET_LINK_LIBRARIES(StereoClient Stereo)

INSTALL(TARGETS StereoClient LIBRARY DESTINATION lib)

#add_subdirectory(gpustereo)

