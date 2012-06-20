include_directories(.)
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

ADD_LIBRARY(cogxmath SHARED cogxmath_rand.cc)
TARGET_LINK_LIBRARIES(cogxmath ${OPENCV_LDFLAGS})
INSTALL(TARGETS cogxmath LIBRARY DESTINATION lib)

