PROJECT(VideoServer)

include_directories(.)
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

# Set the sources.
SET(SOURCES VideoServer.cpp)
SET(HEADERS VideoServer.h)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

# Add libraries to link against.
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CASTCore CDL)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} Video VideoUtils)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)


PROJECT(VideoClient)

INCLUDE_DIRECTORIES(.)

# Set the sources.
SET(SOURCES VideoClient.cpp)
SET(HEADERS VideoClient.h)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

# Add libraries to link against.
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CASTCore CDL)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} Video VideoUtils)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)


# Components!

add_cast_component(OpenCvImgSeqServer OpenCvImgSeqServer.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Video VideoServer VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LIBRARIES}) 

add_cast_component(OpenCvLiveServer OpenCvLiveServer.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Video VideoServer VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LIBRARIES})


