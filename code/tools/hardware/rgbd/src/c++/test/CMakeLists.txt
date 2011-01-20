find_package(OpenCV REQUIRED)

include_directories(../autogen)
include_directories(../components)

add_cast_component(RGBDClient RGBDClient.cpp)
link_cast_component(${CAST_COMPONENT_NAME} KinectDataReceiver RGBD CDL)