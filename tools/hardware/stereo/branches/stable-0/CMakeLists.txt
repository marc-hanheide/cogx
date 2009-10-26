project (Stereo)

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${OPENCV_CFLAGS})
link_directories(${OPENCV_LIBRARY_DIRS})


include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)
include_directories(${COGX_ROOT}/tools/hardware/stereo/src/c++/autogen)

# auto generated code
add_subdirectory(src/c++/autogen)

add_subdirectory(src/c++/components)

