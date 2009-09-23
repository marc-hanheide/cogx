find_package(OpenCV REQUIRED)

include_directories(.)
include_directories(${OPENCV_CFLAGS})
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/autogen)
link_directories (/usr/local/lib)
link_directories (/usr/local/cuda/lib)
link_directories (/usr/local/cuda/sdk/lib)
link_directories (/usr/local/cuda/sdk/C/lib)
link_directories (${COGX_ROOT}/tools/hardware/stereo/src/c++/components/gpustereo)

add_cast_component(StereoServer StereoServer.cpp StereoCamera.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Stereo Video VideoUtils VideoClient)
link_cast_component(${CAST_COMPONENT_NAME} gpustereo)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS})


ADD_LIBRARY(StereoClient SHARED StereoClient.cpp)

# Add libraries to link against.
TARGET_LINK_LIBRARIES(StereoClient CASTCore CDL)
TARGET_LINK_LIBRARIES(StereoClient Stereo)

INSTALL(TARGETS StereoClient LIBRARY DESTINATION lib)

#add_subdirectory(gpustereo)

