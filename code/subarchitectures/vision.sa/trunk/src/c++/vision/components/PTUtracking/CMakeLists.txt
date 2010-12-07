include_directories(.)
include_directories(${COGX_ROOT}/tools/hardware/ptz/src/c++/autogen)
include_directories(${COGX_ROOT}/subarchitectures/spatial.sa/src/c++/autogen)

Find_package(Cure)
include_directories(${CURE_INSTALLROOT}/include/cure)
link_directories(${CURE_INSTALLROOT}/lib/cure)


add_cast_component(PtuTrackPoint PtuTrackPoint.cpp)
link_cast_component(${CAST_COMPONENT_NAME} PTZ Video NavData VisionData)
link_cast_component(${CAST_COMPONENT_NAME} CureTransformation CureMSpace)
