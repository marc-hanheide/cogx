include_directories(.)

Find_package(Cure)
include_directories(${CURE_INSTALLROOT}/include/cure)
link_directories(${CURE_INSTALLROOT}/lib/cure)
include_directories(${COGX_ROOT}/subarchitectures/spatial.sa/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/ptz/src/c++/autogen)

# Set the sources.
SET(SOURCES DisplayConvexHullPB.cpp)

add_cast_component(DisplayConvexHullPB ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} NavData VisionData PTZ ${OPENCV_LDFLAGS})
link_cast_component(${CAST_COMPONENT_NAME} peekabot)
link_cast_component(${CAST_COMPONENT_NAME} cosycure)

