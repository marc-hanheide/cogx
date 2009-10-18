include_directories(../autogen)
include_directories(../)

Find_package(Player)
Find_package(X11)

Find_package(Cure)
include_directories(${CURE_INSTALLROOT}/include/cure)
link_directories(${CURE_INSTALLROOT}/lib/cure)

find_package(Peekabot)
include_directories(${PEEKABOT_INSTALLROOT}/include)



add_cast_component(PullClientLaser PullClientLaser.cpp)
link_cast_component(PullClientLaser Laser LaserClientUtils)

add_cast_component(PushClientLaserRaw PushClientLaserRaw.cpp)
link_cast_component(PushClientLaserRaw Laser)

add_cast_component(PushClientLaser PushClientLaser.cpp)
link_cast_component(PushClientLaser Laser Scan2dReceiver)

