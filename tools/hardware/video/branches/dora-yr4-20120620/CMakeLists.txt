project (Video)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)

# auto generated code
add_subdirectory(src/c++/autogen)
if (DO_SLICE_ONLY)
   return()
endif()

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${OPENCV_CFLAGS})
link_directories(${OPENCV_LIBRARY_DIRS})

add_subdirectory(src/c++/utils)
add_subdirectory(src/c++/components)
add_subdirectory(src/c++/test)

if(BUILD_PYTHON_COMPONENTS)
   add_subdirectory(src/python/slicegen)
endif(BUILD_PYTHON_COMPONENTS)
