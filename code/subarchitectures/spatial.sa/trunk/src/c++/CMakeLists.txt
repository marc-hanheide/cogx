include_directories(../autogen)

Find_package(Player REQUIRED)
Find_package(X11 REQUIRED)

find_package(Peekabot REQUIRED)
include_directories(${PEEKABOT_INSTALLROOT}/include)
link_directories(${PEEKABOT_INSTALLROOT}/lib)


find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})

Find_package(Cure REQUIRED)
include_directories(${CURE_INSTALLROOT}/include/cure)
link_directories(${CURE_INSTALLROOT}/lib/cure)

include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen/)
include_directories(${COGX_ROOT}/tools/hardware/robotbase/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/robotbase/src/c++/autogen/)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils/)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen/)
include_directories(${COGX_ROOT}/tools/hardware/utils/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/ptz/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/autogen)
if(BUILD_SA_BINDING)
include_directories(${COGX_ROOT}/subarchitectures/binder/src/c++/autogen)
include_directories(${COGX_ROOT}/subarchitectures/binder/src/c++/)
endif(BUILD_SA_BINDING)

option(DISPLAY_COMA_PB NO)
if(DISPLAY_COMA_PB)
     add_definitions(-DDISPLAY_COMA)
     include_directories(${COGX_ROOT}/subarchitectures/coma.sa/src/c++)
endif(DISPLAY_COMA_PB)


# -------------- NAV.SA ----------------

option(BUILD_SA_SPATIAL_PEEKABOT "Build spatial.sa PeekaBot components" YES)
if(BUILD_SA_SPATIAL_PEEKABOT)

add_cast_component(DisplayNavInPB DisplayNavInPB.cpp)
link_cast_component(${CAST_COMPONENT_NAME} PTZ NavData VisionData SpatialData Laser Scan2dReceiver SpatialProbabilities SpatialProperties)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} peekabot)
if(DISPLAY_COMA_PB)
  link_cast_component(${CAST_COMPONENT_NAME} FrontierInterface ComaData)
endif(DISPLAY_COMA_PB)
endif(BUILD_SA_SPATIAL_PEEKABOT)

# add_cast_component(PeekabotControl PeekabotControl.cpp)
# link_cast_component(${CAST_COMPONENT_NAME} NavData FrontierInterface)
# link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
# link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})
# link_cast_component(${CAST_COMPONENT_NAME} peekabot)

add_cast_component(SlamProcess SlamProcess.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData Laser Scan2dReceiver Robotbase OdometryReceiver CureHWUtils)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

# -------------- /NAV.SA ----------------

add_cast_component(NavGraphProcess NavGraphProcess.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData Laser Robotbase OdometryReceiver Scan2dReceiver CureHWUtils VisionData)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})


add_cast_component(SpatialTranslation SpatialTranslation.cpp)
link_cast_component(${CAST_COMPONENT_NAME} SpatialData  FrontierInterface)
link_cast_component(${CAST_COMPONENT_NAME} NavData )
link_cast_component(${CAST_COMPONENT_NAME} DLIB)
#link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

add_cast_component(SpatialControl SpatialControl.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData Robotbase Laser Scan2dReceiver OdometryReceiver CureHWUtils RobotbaseClientUtils FrontierInterface)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

add_cast_component(PlaceManager PlaceManager.cpp)
link_cast_component(${CAST_COMPONENT_NAME} SpatialData NavData SpatialProperties FrontierInterface Robotbase Laser Scan2dReceiver OdometryReceiver CureHWUtils RobotbaseClientUtils)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} DLIB)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

add_cast_component(LocalMapManager GridObjectFinder.cpp LocalMapManager.cpp) 
link_cast_component(${CAST_COMPONENT_NAME} SpatialData NavData FrontierInterface Robotbase Laser Scan2dReceiver OdometryReceiver CureHWUtils RobotbaseClientUtils VisionData PTZ)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
#link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS})
link_cast_component(${CAST_COMPONENT_NAME} -lcv -lcxcore)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

#add_cast_component(SpatialPeekabotControl SpatialPeekabotControl.cpp)
#link_cast_component(${CAST_COMPONENT_NAME} NavData Robotbase Laser Scan2dReceiver OdometryReceiver CureHWUtils RobotbaseClientUtils)
#link_cast_component(${CAST_COMPONENT_NAME} SpatialData FrontierInterface)
#link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
#link_cast_component(${CAST_COMPONENT_NAME} peekabot)
#link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

#if(BUILD_SA_BINDING)
#add_cast_component(ProxyMarshaller ProxyMarshaller.cpp)
#link_cast_component(${CAST_COMPONENT_NAME} BinderAbstract)
#link_cast_component(${CAST_COMPONENT_NAME} SpatialData SpatialProperties NavData)
#link_cast_component(${CAST_COMPONENT_NAME} BinderEssentials Marshalling)
#endif(BUILD_SA_BINDING)
#
#if(BUILD_SA_BINDING)
#add_cast_component(ConnectivityWriter ConnectivityWriter.cpp)
#link_cast_component(${CAST_COMPONENT_NAME} SpatialData SpatialProperties)
#link_cast_component(${CAST_COMPONENT_NAME} BinderEssentials Marshalling)
#endif(BUILD_SA_BINDING)

add_cast_component(SelfRepresenter SelfRepresenter.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData)
link_cast_component(${CAST_COMPONENT_NAME} SpatialData SpatialProperties FrontierInterface)

add_cast_component(PathQueryProcessor PathQueryProcessor.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData)
link_cast_component(${CAST_COMPONENT_NAME} SpatialData FrontierInterface)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)

add_cast_component(ExplorationTester ExplorationTester.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData)
link_cast_component(${CAST_COMPONENT_NAME} SpatialData FrontierInterface)
link_cast_component(${CAST_COMPONENT_NAME} DLIB)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})
# add_cast_component(Pusher Pusher.cpp)
# link_cast_component(${CAST_COMPONENT_NAME} NavData)

option(BUILD_AVS NO)

if(BUILD_AVS)

#add_cast_component(AdvObjectSearch AdvObjectSearch.cpp)
#link_cast_component(${CAST_COMPONENT_NAME} peekabot NavData SpatialData VisionData Laser Robotbase RobotbaseClientUtils Scan2dReceiver OdometryReceiver PTZ)
#link_cast_component(${CAST_COMPONENT_NAME} FrontierInterface)
#link_cast_component(${CAST_COMPONENT_NAME} CureHWUtils CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
#link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES} ${OPENCV_LDFLAGS})

Find_package(GTK2 REQUIRED gtk)
include_directories(${GTK2_INCLUDE_DIRS})
link_directories(${GTK2_LIBRARIES})


IF(APPLE)
#nah: the Find_package call doesn't seem to work for me
pkg_search_module(GTK2_FIX REQUIRED gtk+-2.0)
include_directories(${GTK2_FIX_INCLUDE_DIRS})
link_directories(${GTK2_FIX_LIBRARY_DIRS})
ENDIF(APPLE)



  add_cast_component(VisualObjectSearch VisualObjectSearch.cpp DensitySampling.cpp RelationEvaluation.cpp InnessEvaluation.cpp OnnessEvaluation.cpp ConeSlicer.cpp ./AVS/src/AVSPolicyManager.cc)
  link_cast_component(${CAST_COMPONENT_NAME} ${GTK2_LIBRARIES} peekabot NavData SpatialData VisionData Laser Robotbase RobotbaseClientUtils Scan2dReceiver OdometryReceiver PTZ)
  link_cast_component(${CAST_COMPONENT_NAME} FrontierInterface)
  link_cast_component(${CAST_COMPONENT_NAME} CureHWUtils CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES} ${OPENCV_LDFLAGS})

IF(APPLE)
link_cast_component(${CAST_COMPONENT_NAME} ${GTK2_FIX_LIBRARIES})
ENDIF(APPLE)

endif(BUILD_AVS)

add_cast_component(ObjectRelationManager ObjectRelationManager.cpp DensitySampling.cpp RelationEvaluation.cpp OnnessEvaluation.cpp InnessEvaluation.cpp) 
link_cast_component(${CAST_COMPONENT_NAME} SpatialData VisionData SpatialProperties peekabot PTZ FrontierInterface)
link_cast_component(${CAST_COMPONENT_NAME} cv highgui cxcore)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters)

