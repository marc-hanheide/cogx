Find_package(Player)
Find_package(X11)

Find_package(Cure)
include_directories(${CURE_INSTALLROOT}/include/cure)
link_directories(${CURE_INSTALLROOT}/lib/cure)

find_package(Peekabot)
include_directories(${PEEKABOT_INSTALLROOT}/include)

add_cast_component(LaserStop LaserStop.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Robotbase OdometryReceiver Scan2dReceiver)


