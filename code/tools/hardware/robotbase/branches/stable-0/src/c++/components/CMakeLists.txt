include_directories(../autogen)

Find_package(Player)
Find_package(X11)

Find_package(Cure)
include_directories(${CURE_INSTALLROOT}/include/cure)
link_directories(${CURE_INSTALLROOT}/lib/cure)

find_package(Peekabot)
include_directories(${PEEKABOT_INSTALLROOT}/include)

add_cast_component(JoystickDrivable JoystickDrivable.cpp)
link_cast_component(JoystickDrivable CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX CureHalDeviceBase CureHalHardwareUtils CureHalRobotP2OS CureHalJoystick)

add_cast_component(RobotbaseServerPlayer RobotbaseServerPlayer.cpp)
link_cast_component(${CAST_COMPONENT_NAME} ${PLAYER_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} Robotbase JoystickDrivable ${X11_LIBRARIES})

add_cast_component(RobotbaseServerCure RobotbaseServerCure.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Robotbase JoystickDrivable)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX CureHalDeviceBase CureHalHardwareUtils CureHalRobotP2OS CureHalJoystick)

# Set the sources.zbI			   
SET(SOURCES RobotbaseClientUtils.cpp)
SET(HEADERS RobotbaseClientUtils.hpp)
ADD_LIBRARY(RobotbaseClientUtils SHARED ${SOURCES})
# Add libraries to link against.
target_link_libraries(RobotbaseClientUtils ${PLAYER_LIBRARIES})
target_link_libraries(RobotbaseClientUtils Robotbase)
target_link_libraries(RobotbaseClientUtils ${CAST_LIBRARIES})
target_link_libraries(RobotbaseClientUtils ${ICE_LIBS})
INSTALL(TARGETS RobotbaseClientUtils LIBRARY DESTINATION lib)

SET(SOURCES OdometryReceiver.cpp)
SET(HEADERS OdometryReceiver.hpp)
ADD_LIBRARY(OdometryReceiver SHARED ${SOURCES})
# Add libraries to link against.
target_link_libraries(OdometryReceiver Robotbase)
target_link_libraries(OdometryReceiver ${CAST_LIBRARIES})
target_link_libraries(OdometryReceiver ${ICE_LIBS})

INSTALL(TARGETS OdometryReceiver LIBRARY DESTINATION lib)

option(BUILD_HAL_ROBOT_TESTS "Build hardware robot tests" NO)
if(BUILD_HAL_ROBOT_TESTS)
add_subdirectory (tests)
endif(BUILD_HAL_ROBOT_TESTS)

ADD_EXECUTABLE(TourGuide TourGuide.cpp)
target_link_libraries(TourGuide ${PLAYER_LIBRARIES})