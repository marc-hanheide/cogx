include_directories(../autogen)

Find_package(Player)
Find_package(X11)

Find_package(Cure)
include_directories(${CURE_INSTALLROOT}/include/cure)
link_directories(${CURE_INSTALLROOT}/lib/cure)

find_package(Peekabot)
include_directories(${PEEKABOT_INSTALLROOT}/include)


add_cast_component(LaserServerPlayer LaserServerPlayer.cpp)
target_link_libraries(LaserServerPlayer ${PLAYER_LIBRARIES})
target_link_libraries(LaserServerPlayer Laser)



# Set the sources.
SET(SOURCES LaserClientUtils.cpp)
SET(HEADERS LaserClientUtils.hpp)
ADD_LIBRARY(LaserClientUtils SHARED ${SOURCES})
# Add libraries to link against.
TARGET_LINK_LIBRARIES(LaserClientUtils Laser)
TARGET_LINK_LIBRARIES(LaserClientUtils CDL CASTCore)
TARGET_LINK_LIBRARIES(LaserClientUtils ${ICE_LIBS})
INSTALL(TARGETS LaserClientUtils LIBRARY DESTINATION lib)

# Set the sources.
SET(SOURCES Scan2dReceiver.cpp)
SET(HEADERS Scan2dReceiver.hpp)
ADD_LIBRARY(Scan2dReceiver SHARED ${SOURCES})
# Add libraries to link against.
TARGET_LINK_LIBRARIES(Scan2dReceiver Laser)
TARGET_LINK_LIBRARIES(Scan2dReceiver CASTCore CDL)
INSTALL(TARGETS Scan2dReceiver LIBRARY DESTINATION lib)

option(BUILD_HAL_LASER_TESTS "Build hardware laser tests" NO)
if(BUILD_HAL_LASER_TESTS)
add_subdirectory (tests)
endif(BUILD_HAL_LASER_TESTS)
