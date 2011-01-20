find_package(OpenCV REQUIRED)

include_directories(.)
include_directories(../autogen)
link_directories (/usr/local/lib)
include_directories(/usr/include/ni)

add_cast_component(RGBDServer RGBDServer.cpp)
link_cast_component(${CAST_COMPONENT_NAME} RGBD CDL XnCore)

SET(SOURCES KinectDataReceiver.cpp)
SET(HEADERS KinectDataReceiver.h)
ADD_LIBRARY(KinectDataReceiver SHARED ${SOURCES})
# Add libraries to link against.

target_link_libraries(KinectDataReceiver ${CAST_LIBRARIES} RGBD)
target_link_libraries(KinectDataReceiver ${ICE_LIBS})

INSTALL(TARGETS KinectDataReceiver LIBRARY DESTINATION lib)