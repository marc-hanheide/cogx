Find_package(Cure REQUIRED)
include_directories(${CURE_INSTALLROOT}/include/cure)
link_directories(${CURE_INSTALLROOT}/lib/cure)

add_cast_component(DataSaver DataSaver.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Laser Scan2dReceiver Robotbase OdometryReceiver PointCloudClient PointCloud)
link_cast_component(${CAST_COMPONENT_NAME} boost_filesystem)