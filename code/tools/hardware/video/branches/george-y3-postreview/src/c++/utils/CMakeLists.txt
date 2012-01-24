FIND_PACKAGE( OpenCV REQUIRED )

INCLUDE_DIRECTORIES(${COGX_ROOT}/tools/math/src/c++/autogen)
INCLUDE_DIRECTORIES(${COGX_ROOT}/tools/math/src/c++/math)

ADD_LIBRARY(VideoUtils SHARED
   CDataFile.cpp
   CameraParameters.cpp
   ConvertImage.cpp
   ImageCache.cpp
   VideoClient2.cpp
   Timers.cpp
   )

# Add libraries to link against.
TARGET_LINK_LIBRARIES(VideoUtils CASTCore CDL)
TARGET_LINK_LIBRARIES(VideoUtils Video)
TARGET_LINK_LIBRARIES(VideoUtils Math)
TARGET_LINK_LIBRARIES(VideoUtils ${OPENCV_LDFLAGS})

INSTALL(TARGETS VideoUtils LIBRARY DESTINATION lib)

