include_directories(../autogen)

Find_package(Player REQUIRED)
Find_package(X11 REQUIRED)
# Set build type
# set(CMAKE_BUILD_TYPE Debug)

find_package(Peekabot REQUIRED)
include_directories(${PEEKABOT_INSTALLROOT}/include)
link_directories(${PEEKABOT_INSTALLROOT}/lib)

# Find QT4 
find_package( Qt4 REQUIRED )
include(${QT_USE_FILE})

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIRS})

find_package(PkgConfig)

if (NOT APPLE)
	set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-as-needed")
endif (NOT APPLE)

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
#MESSAGE(${OPENCV_INCLUDE_DIRS})
#MESSAGE(${OPENCV_LDFLAGS})

#find_package(OpenNI)
#include_directories(${OPENNI_INCLUDE_DIRS})
include_directories(/usr/include/ni)

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
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/rocs/src/concept/cpp/)
include_directories(${COGX_ROOT}/tools/rocs/src/ml/cpp/)
include_directories(${COGX_ROOT}/tools/rocs/src/core/cpp/)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/autogen)
include_directories(scanmap/HSS)


include_directories(${COGX_ROOT}/subarchitectures/conceptual.sa/src/c++/conceptual/shared/)

include_directories(${COGX_ROOT}/tools/cogxutils/src/autogen)

include_directories(${COGX_ROOT}/subarchitectures/coma.sa/src/c++/)

include_directories(${COGX_ROOT}/subarchitectures/conceptual.sa/src/c++/conceptual/autogen)

include_directories(${COGX_ROOT}/tools/cogxutils/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs+cast/src/cpp/autogen)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs/src/cpp/autogen)

include_directories(/home/krsj/code/libdai/include)
link_directories(/home/krsj/code/libdai/lib)


if(BUILD_SA_BINDING)
include_directories(${COGX_ROOT}/subarchitectures/binder/src/c++/autogen)
include_directories(${COGX_ROOT}/subarchitectures/binder/src/c++/)
endif(BUILD_SA_BINDING)

option(DISPLAY_COMA_PB YES)
if(DISPLAY_COMA_PB)
     add_definitions(-DDISPLAY_COMA)
     include_directories(${COGX_ROOT}/subarchitectures/coma.sa/src/c++)
endif(DISPLAY_COMA_PB)


# -------------- NAV.SA ----------------

option(BUILD_SA_SPATIAL_PEEKABOT "Build spatial.sa PeekaBot components" YES)
if(BUILD_SA_SPATIAL_PEEKABOT)

include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)


add_cast_component(DisplayNavInPB DisplayNavInPB.cpp)
link_cast_component(${CAST_COMPONENT_NAME} ${GTKMM_LIBRARIES} PTZ NavData VisionData SpatialData ComaData ConceptualData SpatialProbabilities SpatialProperties)
#link_cast_component(${CAST_COMPONENT_NAME} PointCloud)
link_cast_component(${CAST_COMPONENT_NAME} CureTransformation CureSensory CureAddressBank CureFilters CureNav CureMapApps CureMSpace CureNavX CureMapX CureMapBase CureMapWall CureMapPoint CureMapHLine CureMapRangedPoint CureRelativePointFeature CureSensorData CureGeometry CureMath CureUtils)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} peekabot)
link_cast_component(${CAST_COMPONENT_NAME} FrontierInterface)
link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)

if(DISPLAY_COMA_PB)
  link_cast_component(${CAST_COMPONENT_NAME} ComaData)
endif(DISPLAY_COMA_PB)
endif(BUILD_SA_SPATIAL_PEEKABOT)

# add_cast_component(PeekabotControl PeekabotControl.cpp)
# link_cast_component(${CAST_COMPONENT_NAME} NavData FrontierInterface)
# link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
# link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})
# link_cast_component(${CAST_COMPONENT_NAME} peekabot)

add_cast_component(SlamProcess SlamProcess.cpp)
link_cast_component(${CAST_COMPONENT_NAME} peekabot CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapBase CureMapWall CureMapPoint CureMapHLine CureMapRangedPoint CureRelativePointFeature CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} NavData Laser Scan2dReceiver Robotbase OdometryReceiver CureHWUtils)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

# -------------- /NAV.SA ----------------

add_cast_component(NavGraphProcess NavGraphProcess.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavData Laser Robotbase OdometryReceiver Scan2dReceiver CureHWUtils VisionData FrontierInterface)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})


add_cast_component(SpatialTranslation SpatialTranslation.cpp)
link_cast_component(${CAST_COMPONENT_NAME} SpatialData  FrontierInterface)
link_cast_component(${CAST_COMPONENT_NAME} NavData )
link_cast_component(${CAST_COMPONENT_NAME} DLIB)
#link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

add_cast_component(SpatialControl SpatialControl.cpp NewNavController.cpp NewTurnMotion.cpp)
link_cast_component(${CAST_COMPONENT_NAME} peekabot PTZ NavData Robotbase Laser Scan2dReceiver OdometryReceiver CureHWUtils RobotbaseClientUtils FrontierInterface DLIB)
link_cast_component(${CAST_COMPONENT_NAME} CureTransformation CureAddressBank CureMapApps CureNav CureNavX CureMapX CureMapBase CureMapWall CureMapHLine CureMapRangedPoint CureMapPoint CureRelativePointFeature CureGeometry CureFilters CureSensory CureSensorData CureUtils CureMath CureMSpace)
link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

add_cast_component(PlaceManager PlaceManager.cpp)
link_cast_component(${CAST_COMPONENT_NAME} SpatialData NavData SpatialProperties FrontierInterface Robotbase Laser Scan2dReceiver OdometryReceiver CureHWUtils RobotbaseClientUtils)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
link_cast_component(${CAST_COMPONENT_NAME} DLIB)
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

add_cast_component(LocalMapManager GridObjectFinder.cpp LocalMapManager.cpp scanmap/HSS/HSSDoorExtractor.cc scanmap/HSS/HSSDisplayFunctions.cc scanmap/HSS/HSSLsqLineExtractor.cc scanmap/HSS/HSSRedundantLine2DRep.cc) 
link_cast_component(${CAST_COMPONENT_NAME} SpatialData NavData FrontierInterface Robotbase Laser Scan2dReceiver OdometryReceiver CureHWUtils RobotbaseClientUtils VisionData PTZ)
link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)

link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS})
link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES})

link_cast_component(${CAST_COMPONENT_NAME} peekabot)
 link_cast_component(${CAST_COMPONENT_NAME} ${OpenCV_LIBRARIES} opencv_core)
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

Find_package(GTK2 REQUIRED gtk)
include_directories(${GTK2_INCLUDE_DIRS})
link_directories(${GTK2_LIBRARIES})



#nah: the Find_package call doesn't seem to work for me.
#marc: same here, neither on Apple nor on ubuntu maverick this works, pkg-search-module does the job:
pkg_search_module(GTK2_FIX REQUIRED gtk+-2.0)
include_directories(${GTK2_FIX_INCLUDE_DIRS})
link_directories(${GTK2_FIX_LIBRARY_DIRS})



  add_cast_component(VisualObjectSearch VisualObjectSearch.cpp DensitySampling.cpp RelationEvaluation.cpp InnessEvaluation.cpp OnnessEvaluation.cpp ConeSlicer.cpp ./AVS/src/AVSPolicyManager.cc)
  link_cast_component(${CAST_COMPONENT_NAME} ${GTK2_LIBRARIES} peekabot NavData SpatialData VisionData Laser Robotbase RobotbaseClientUtils Scan2dReceiver OdometryReceiver PTZ)
  link_cast_component(${CAST_COMPONENT_NAME} FrontierInterface)
  link_cast_component(${CAST_COMPONENT_NAME} CureHWUtils CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
  link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)
	link_cast_component(${CAST_COMPONENT_NAME} ${X11_LIBRARIES} ${OPENCV_LDFLAGS})

IF(APPLE)
link_cast_component(${CAST_COMPONENT_NAME} ${GTK2_FIX_LIBRARIES})
ENDIF(APPLE)

endif(BUILD_AVS)

option(BUILD_SA_SPATIAL_RELATIONMANAGER "Build spatial.SA ObjectRelationManager component" NO)
if(BUILD_SA_SPATIAL_RELATIONMANAGER)
  include_directories(${COGX_ROOT}/tools/rocs/thirdparty/libdai/include)
  add_cast_component(ObjectRelationManager ObjectRelationManager.cpp DensitySampling.cpp RelationEvaluation.cpp OnnessEvaluation.cpp InnessEvaluation.cpp)# InferRelations.cpp) 

  set_target_properties(${CAST_COMPONENT_NAME} PROPERTIES INSTALL_RPATH ${ROCS_DIR}/lib)
  link_cast_component(${CAST_COMPONENT_NAME} dai rocs_core rocs_ml rocs_concept)

  link_cast_component(${CAST_COMPONENT_NAME} SpatialData VisionData SpatialProperties peekabot PTZ FrontierInterface)
  link_cast_component(${CAST_COMPONENT_NAME} ${OpenCV_LIBRARIES})
  link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters)

endif(BUILD_SA_SPATIAL_RELATIONMANAGER)

option(BUILD_SA_SPATIAL_RELATIONTESTER "Build spatial.SA ObjectRelationTester component" NO)

if(BUILD_SA_SPATIAL_RELATIONTESTER)
  add_cast_component(ObjectRelationTester ObjectRelationTester.cpp DensitySampling.cpp RelationEvaluation.cpp OnnessEvaluation.cpp InnessEvaluation.cpp) 
  set_target_properties(${CAST_COMPONENT_NAME} PROPERTIES INSTALL_RPATH ${ROCS_DIR}/lib)
  link_cast_component(${CAST_COMPONENT_NAME} peekabot)
  link_cast_component(${CAST_COMPONENT_NAME} CureUtils CureAddressBank CureTransformation CureMath CureSensorData)
  link_cast_component(${CAST_COMPONENT_NAME} Math)
endif(BUILD_SA_SPATIAL_RELATIONTESTER)

OPTION(BUILD_AVS_PLANNER NO)
if(BUILD_AVS_PLANNER)
    add_cast_component(AVS_ContinualPlanner AVS_ContinualPlanner.cpp DensitySampling.cpp RelationEvaluation.cpp InnessEvaluation.cpp OnnessEvaluation.cpp ConeSlicer.cpp ViewPointGenerator.cpp) 
    link_cast_component(${CAST_COMPONENT_NAME} peekabot PTZ)
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualShared NavData VisionData ConceptualData FrontierInterface SpatialProperties SpatialProbabilities SpatialData CureAddressBank ComaData BeliefsCogX )
# link_cast_component(${CAST_COMPONENT_NAME} cv highgui cxcore)
 link_cast_component(${CAST_COMPONENT_NAME} ${OpenCV_LIBRARIES} opencv_core)
   link_cast_component(${CAST_COMPONENT_NAME} CureHWUtils CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapApps CureNavX CureMapX)
endif(BUILD_AVS_PLANNER)

add_cast_component(PlaceholderExplorer PlaceholderExplorer.cpp)
link_cast_component(${CAST_COMPONENT_NAME} SpatialData FrontierInterface)
