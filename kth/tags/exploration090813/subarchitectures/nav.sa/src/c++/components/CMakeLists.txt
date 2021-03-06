include_directories(../autogen)

Find_package(Player)
Find_package(X11)

Find_package(Cure)
include_directories(${CURE_INSTALLROOT}/include/cure)
link_directories(${CURE_INSTALLROOT}/lib/cure)


find_package(Peekabot)
include_directories(${PEEKABOT_INSTALLROOT}/include)
link_directories(${PEEKABOT_INSTALLROOT}/lib)

include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen/)
include_directories(${COGX_ROOT}/tools/hardware/robotbase/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/robotbase/src/c++/autogen/)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen/)
include_directories(${COGX_ROOT}/tools/hardware/utils/src/c++/components/)

option(BUILD_SA_NAV_PEEKABOT "Build peekabot nav.sa comp" YES)
if(BUILD_SA_NAV_PEEKABOT)

add_cast_component(DisplayNavInPB DisplayNavInPB.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData Laser Scan2dReceiver)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} peekabot)

add_cast_component(PeekabotControl PeekabotControl.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} peekabot)

endif(BUILD_SA_NAV_PEEKABOT)

add_cast_component(SlamProcess SlamProcess.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData Laser Scan2dReceiver Robotbase OdometryReceiver CureHWUtils)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

add_cast_component(TranslationProcess TranslationProcess.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData )
link_cast_component(${CAST_COMPONENT_NAME} DLIB)
#link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

add_cast_component(NavGraphProcess NavGraphProcess.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData Laser Robotbase OdometryReceiver Scan2dReceiver CureHWUtils)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

add_cast_component(CureDataLogger CureDataLogger.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData Laser Robotbase OdometryReceiver Scan2dReceiver CureHWUtils)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)

add_cast_component(NavControl NavControl.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData Robotbase Laser Scan2dReceiver OdometryReceiver CureHWUtils RobotbaseClientUtils)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

add_cast_component(KeyCommander KeyCommander.cpp)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils)
link_cast_component(${CAST_COMPONENT_NAME} NavData)

add_cast_component(OdomLocalizer OdomLocalizer.cpp)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureAddressBank CureTransformation CureMath CureSensorData)
link_cast_component(${CAST_COMPONENT_NAME} NavData Robotbase OdometryReceiver CureHWUtils)

# add_cast_component(Pusher Pusher.cpp)
# link_cast_component(${CAST_COMPONENT_NAME} NavData)

option(BUILD_SA_NAV_OBJECTSEARCH "Build objectsearch nav.sa comp" NO)
if(BUILD_SA_NAV_OBJECTSEARCH)
set(SOURCES ObjPdf.cpp)
add_cast_component(ObjectSearch ObjectSearch.cpp ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} NavData Robotbase Laser Scan2dReceiver OdometryReceiver CureHWUtils RobotbaseClientUtils) 
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})
endif(BUILD_SA_NAV_OBJECTSEARCH)