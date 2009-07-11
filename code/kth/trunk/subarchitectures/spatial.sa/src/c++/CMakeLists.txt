include_directories(../autogen)

Find_package(Player)
Find_package(X11)

Find_package(Cure)
include_directories(${CURE_INSTALLROOT}/include/cure)
link_directories(${CURE_INSTALLROOT}/lib/cure)

include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen/)
include_directories(${COGX_ROOT}/tools/hardware/robotbase/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/robotbase/src/c++/autogen/)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen/)
include_directories(${COGX_ROOT}/tools/hardware/utils/src/c++/components/)
include_directories(${COGX_ROOT}/subarchitectures/nav.sa/src/c++/autogen)

add_cast_component(SpatialTranslation SpatialTranslation.cpp)
link_cast_component(${CAST_COMPONENT_NAME} SpatialData )
link_cast_component(${CAST_COMPONENT_NAME} NavData )
link_cast_component(${CAST_COMPONENT_NAME} DLIB)
#link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

add_cast_component(SpatialControl SpatialControl.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData Robotbase Laser Scan2dReceiver OdometryReceiver CureHWUtils RobotbaseClientUtils FrontierInterface)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

add_cast_component(PlaceManager PlaceManager.cpp)
link_cast_component(${CAST_COMPONENT_NAME} SpatialData NavData Robotbase Laser Scan2dReceiver OdometryReceiver CureHWUtils RobotbaseClientUtils)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

add_cast_component(SpatialPeekabotControl SpatialPeekabotControl.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData Robotbase Laser Scan2dReceiver OdometryReceiver CureHWUtils RobotbaseClientUtils)
link_cast_component(${CAST_COMPONENT_NAME} SpatialData)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} peekabot)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

add_cast_component(PathQueryProcessor PathQueryProcessor.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData)
link_cast_component(${CAST_COMPONENT_NAME} SpatialData)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)

add_cast_component(ExplorationTester ExplorationTester.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData)
link_cast_component(${CAST_COMPONENT_NAME} SpatialData)
link_cast_component(${CAST_COMPONENT_NAME} DLIB)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})
# add_cast_component(Pusher Pusher.cpp)
# link_cast_component(${CAST_COMPONENT_NAME} NavData)


