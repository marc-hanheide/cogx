include_directories(.)
include_directories(/opt/local/include)
link_directories (/opt/local/lib)

# Set the sources.
SET(SOURCES VideoServer.cpp)
SET(HEADERS VideoServer.h)

add_library(VideoServer SHARED ${SOURCES})
target_link_libraries(VideoServer CASTCore CDL)
target_link_libraries(VideoServer Video VideoUtils)

INSTALL(TARGETS VideoServer LIBRARY DESTINATION lib)



add_cast_component(OpenCvImgSeqServer OpenCvImgSeqServer.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Video VideoServer VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS})

add_cast_component(OpenCvLiveServer OpenCvLiveServer.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Video VideoServer VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS})


