Find_package(Player)
Find_package(X11)

Find_package(Cure)
include_directories(${CURE_INSTALLROOT}/include/cure)
link_directories(${CURE_INSTALLROOT}/lib/cure)

find_package(Peekabot)
include_directories(${PEEKABOT_INSTALLROOT}/include)

# Set the sources.
SET(SOURCES CureHWUtils.cpp)
SET(HEADERS CureHWUtils.hpp)
ADD_LIBRARY(CureHWUtils SHARED ${SOURCES})
TARGET_LINK_LIBRARIES(CureHWUtils Laser Robotbase)
TARGET_LINK_LIBRARIES(CureHWUtils Ice IceUtil)
TARGET_LINK_LIBRARIES(CureHWUtils CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank)
INSTALL(TARGETS CureHWUtils LIBRARY DESTINATION lib)


add_cast_component(LaserRobotServerDummy LaserRobotServerDummy.cpp)
link_cast_component(LaserRobotServerDummy Laser Robotbase)
