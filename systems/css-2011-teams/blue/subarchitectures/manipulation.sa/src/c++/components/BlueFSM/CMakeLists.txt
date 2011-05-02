
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

include_directories(${VISUALIZATION_INCLUDE_DIRS})

include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/autogen)
include_directories(${COGX_ROOT}/subarchitectures/spatial.sa/src/c++/)
include_directories(${COGX_ROOT}/subarchitectures/spatial.sa/src/c++/autogen)
include_directories(${CURE_INSTALLROOT}/include/cure)

Find_package(Player)
FIND_PACKAGE( OpenCV REQUIRED )
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

include_directories(WildMagic5p4)
add_cast_component(BlueFSM BlueFSM.cpp WildMagic5p4/Wm5Assert.cpp WildMagic5p4/Wm5CorePCH.cpp WildMagic5p4/Wm5EigenDecomposition.cpp WildMagic5p4/Wm5Math.cpp WildMagic5p4/Wm5MathematicsPCH.cpp WildMagic5p4/Wm5Matrix2.cpp WildMagic5p4/Wm5Matrix3.cpp WildMagic5p4/Wm5Matrix4.cpp WildMagic5p4/Wm5Memory.cpp WildMagic5p4/Wm5Mutex.cpp WildMagic5p4/Wm5Quaternion.cpp WildMagic5p4/Wm5SingularValueDecomposition.cpp WildMagic5p4/Wm5Vector2.cpp WildMagic5p4/Wm5Vector3.cpp WildMagic5p4/Wm5Vector4.cpp)
link_cast_component(${CAST_COMPONENT_NAME} ${VISUALIZATION_LIBRARIES} ${PLAYER_LDFLAGS} ${OPENCV_LDFLAGS}) 

target_link_libraries(BlueFSM VisionData ManipulationData)
target_link_libraries(BlueFSM VisionData FrontierInterface)
