cmake_minimum_required(VERSION 2.6)
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


# Golem
PROJECT(Golem)

# Project root directory
GET_FILENAME_COMPONENT(GOLEM_ROOT ../../.. ABSOLUTE CACHE INTERNAL "Path prefix for Golem package")

# Set the path where other thing should be relative to
GET_FILENAME_COMPONENT(CMAKE_SOURCE_DIR . ABSOLUTE CACHE INTERNAL "")

# Single output directory for building all Golem libraries
SET(LIBRARY_OUTPUT_PATH ${GOLEM_ROOT}/lib/linux CACHE PATH "Single output directory for building all Golem libraries")

# Single output directory for building all Golem executables
SET(EXECUTABLE_OUTPUT_PATH ${GOLEM_ROOT}/bin/linux CACHE PATH "Single output directory for building all Golem executables")

# Set the install directory.
SET(CMAKE_INSTALL_PREFIX /usr/local CACHE PATH "Path prefix for Golem installation")

SET(ICE_INCLUDE /usr/include CACHE PATH "Path prefix for ZeroC Ice include")
SET(ICE_LIBRARY /usr/lib CACHE PATH "Path prefix for ZeroC Ice library")
SET(ICE_BIN /usr/bin CACHE PATH "Path prefix for ZeroC Ice binaries")

SET(PHYSX_INCLUDE /usr/include/PhysX/v2.8.3 CACHE PATH "Path prefix for PhysX include")
SET(PHYSX_LIBRARY /usr/lib/PhysX/v2.8.3 CACHE PATH "Path prefix for PhysX library")

option(BUILD_32_BIT "compile for 32 instead of 64 bit" NO)
if(BUILD_32_BIT)
  SET(CMAKE_CXX_FLAGS ${CXX_FLAGS} "-O2 -m32 -DLINUX -DNX32")
else(BUILD_32_BIT)
  SET(CMAKE_CXX_FLAGS ${CXX_FLAGS} "-O2 -fPIC -DLINUX -DNX64")
endif(BUILD_32_BIT)

FIND_PACKAGE(Boost COMPONENTS thread)
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

option(BUILD_DEFS "Build Golem Defs library" YES)
option(BUILD_MATH "Build Golem Math library" YES)
option(BUILD_SYS "Build Golem Sys library" YES)
option(BUILD_TOOLS "Build Golem Tools library" YES)
option(BUILD_CTRL "Build Golem Ctrl library" YES)
option(BUILD_PHYS "Build Golem Phys library" YES)
option(BUILD_PHYS_CTRL "Build Golem PhysCtrl library" YES)
option(BUILD_DEMO_COMMON "Build Golem example programs library" YES)
option(BUILD_DEMO "Build Golem example programs" YES)
option(BUILD_AXNI "Build Golem AxNI test" YES)
option(BUILD_TINY "Build Golem Tiny library" YES)
option(BUILD_TINY_DEMO "Build Golem Tiny example program" YES)
option(BUILD_TINY_ICE "Build Golem TinyIce server" YES)
option(BUILD_TINY_ICE_DEMO "Build Golem TinyIce example program" YES)
option(BUILD_DEVICE_BUFCTRL "Build Golem Device BufCtrl library" YES)
option(BUILD_DEVICE_BUFCTRLSIM "Build Golem device bufctrlsim library" YES)
option(BUILD_DEVICE_KATANA "Build Golem Device Katana common library" YES)
option(BUILD_DEVICE_KATANA300 "Build Golem Device Katana300 library" YES)
option(BUILD_DEVICE_KATANA300SIM "Build Golem Device Katana300Sim library" YES)
option(BUILD_DEVICE_KATANA450 "Build Golem Device Katana450 library" YES)
option(BUILD_DEVICE_SIXAXISSIM "Build Golem Device sixAxisSim library" YES)


INCLUDE_DIRECTORIES(
	${GOLEM_ROOT}/include
	${GOLEM_ROOT}/resources
	${GOLEM_ROOT}/include/Golem/Demo
	${ICE_INCLUDE}
	${PHYSX_INCLUDE}/SDKs/Physics/include
	${PHYSX_INCLUDE}/SDKs/Foundation/include
	${PHYSX_INCLUDE}/SDKs/PhysXLoader/include
	${PHYSX_INCLUDE}/SDKs/Cooking/include
)

LINK_DIRECTORIES(
	${ICE_LIBRARY}
	${PHYSX_LIBRARY}
)


###############################################################################
#
# Golem defs library
#
###############################################################################

if(BUILD_DEFS)
SET(DEFS_SOURCES
	${GOLEM_ROOT}/src/Golem/Defs/Constants.cpp
)
SET(DEFS_HEADERS
	${GOLEM_ROOT}/include/Golem/Defs/Assert.h
	${GOLEM_ROOT}/include/Golem/Defs/Constants.h
	${GOLEM_ROOT}/include/Golem/Defs/Desc.h
	${GOLEM_ROOT}/include/Golem/Defs/Pointers.h
	${GOLEM_ROOT}/include/Golem/Defs/System.h
	${GOLEM_ROOT}/include/Golem/Defs/Types.h
)
ADD_LIBRARY(GolemDefs SHARED ${DEFS_SOURCES})
INSTALL(TARGETS GolemDefs LIBRARY DESTINATION lib)
INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/Golem/Defs/)
endif(BUILD_DEFS)

###############################################################################
#
# Golem math library
#
###############################################################################

if(BUILD_MATH)
SET(MATH_SOURCES
	${GOLEM_ROOT}/src/Golem/Math/Bounds.cpp
	${GOLEM_ROOT}/src/Golem/Math/GraphSearch.cpp
	${GOLEM_ROOT}/src/Golem/Math/Mat33.cpp
	${GOLEM_ROOT}/src/Golem/Math/Mat34.cpp
	${GOLEM_ROOT}/src/Golem/Math/Rand.cpp
	${GOLEM_ROOT}/src/Golem/Math/TriangleMesh.cpp
)
SET(MATH_HEADERS
	${GOLEM_ROOT}/include/Golem/Math/Bounds.h
	${GOLEM_ROOT}/include/Golem/Math/Collection.h
	${GOLEM_ROOT}/include/Golem/Math/Derivative.h
	${GOLEM_ROOT}/include/Golem/Math/Extremum.h
	${GOLEM_ROOT}/include/Golem/Math/Function.h
	${GOLEM_ROOT}/include/Golem/Math/GraphSearch.h
	${GOLEM_ROOT}/include/Golem/Math/Mat22.h
	${GOLEM_ROOT}/include/Golem/Math/Mat23.h
	${GOLEM_ROOT}/include/Golem/Math/Mat33.h
	${GOLEM_ROOT}/include/Golem/Math/Mat34.h
	${GOLEM_ROOT}/include/Golem/Math/Math.h
	${GOLEM_ROOT}/include/Golem/Math/Node.h
	${GOLEM_ROOT}/include/Golem/Math/Quat.h
	${GOLEM_ROOT}/include/Golem/Math/Queue.h
	${GOLEM_ROOT}/include/Golem/Math/Rand.h
	${GOLEM_ROOT}/include/Golem/Math/Root.h
	${GOLEM_ROOT}/include/Golem/Math/Sample.h
	${GOLEM_ROOT}/include/Golem/Math/TriangleMesh.h
	${GOLEM_ROOT}/include/Golem/Math/Twist.h
	${GOLEM_ROOT}/include/Golem/Math/Vec2.h
	${GOLEM_ROOT}/include/Golem/Math/Vec3.h
	${GOLEM_ROOT}/include/Golem/Math/VecN.h
	${GOLEM_ROOT}/include/Golem/Math/Wrench.h
)
ADD_LIBRARY(GolemMath SHARED ${MATH_SOURCES})
INSTALL(TARGETS GolemMath LIBRARY DESTINATION lib)
INSTALL(FILES ${MATH_HEADERS} DESTINATION include/Golem/Math/)
endif(BUILD_MATH)

###############################################################################
#
# Golem sys library
#
###############################################################################

if(BUILD_SYS)
SET(SYS_SOURCES
	${GOLEM_ROOT}/src/Golem/Sys/Thread.cpp
	${GOLEM_ROOT}/src/Golem/Sys/Timer.cpp
)
SET(SYS_HEADERS
	${GOLEM_ROOT}/include/Golem/Sys/Thread.h
	${GOLEM_ROOT}/include/Golem/Sys/Timer.h
)
ADD_LIBRARY(GolemSys SHARED ${SYS_SOURCES})
TARGET_LINK_LIBRARIES(GolemSys pthread rt)
INSTALL(TARGETS GolemSys LIBRARY DESTINATION lib)
INSTALL(FILES ${SYS_HEADERS} DESTINATION include/Golem/Sys/)
endif(BUILD_SYS)

###############################################################################
#
# Golem tools library
#
###############################################################################

if(BUILD_TOOLS)
SET(TOOLS_SOURCES
	${GOLEM_ROOT}/src/Golem/Tools/Context.cpp
	${GOLEM_ROOT}/src/Golem/Tools/Data.cpp
	${GOLEM_ROOT}/src/Golem/Tools/Library.cpp
	${GOLEM_ROOT}/src/Golem/Tools/Message.cpp
	${GOLEM_ROOT}/src/Golem/Tools/MessageStream.cpp
	${GOLEM_ROOT}/src/Golem/Tools/Parallels.cpp
	${GOLEM_ROOT}/src/Golem/Tools/Stream.cpp
	${GOLEM_ROOT}/src/Golem/Tools/XMLData.cpp
	${GOLEM_ROOT}/src/Golem/Tools/XMLParser.cpp
)
SET(TOOLS_HEADERS
	${GOLEM_ROOT}/include/Golem/Tools/Context.h
	${GOLEM_ROOT}/include/Golem/Tools/Data.h
	${GOLEM_ROOT}/include/Golem/Tools/Debug.h
	${GOLEM_ROOT}/include/Golem/Tools/Library.h
	${GOLEM_ROOT}/include/Golem/Tools/Message.h
	${GOLEM_ROOT}/include/Golem/Tools/MessageStream.h
	${GOLEM_ROOT}/include/Golem/Tools/Msg.h
	${GOLEM_ROOT}/include/Golem/Tools/Parallels.h
	${GOLEM_ROOT}/include/Golem/Tools/Stream.h
	${GOLEM_ROOT}/include/Golem/Tools/XMLData.h
	${GOLEM_ROOT}/include/Golem/Tools/XMLParser.h
)
ADD_LIBRARY(GolemTools SHARED ${TOOLS_SOURCES})
TARGET_LINK_LIBRARIES(GolemTools GolemDefs GolemMath GolemSys expat)
INSTALL(TARGETS GolemTools LIBRARY DESTINATION lib)
INSTALL(FILES ${TOOLS_HEADERS} DESTINATION include/Golem/Tools/)
endif(BUILD_TOOLS)

###############################################################################
#
# Golem ctrl library
#
###############################################################################

if(BUILD_CTRL)
SET(CTRL_SOURCES
	${GOLEM_ROOT}/src/Golem/Ctrl/Arm.cpp
	${GOLEM_ROOT}/src/Golem/Ctrl/Data.cpp
	${GOLEM_ROOT}/src/Golem/Ctrl/DEKinematics.cpp
	${GOLEM_ROOT}/src/Golem/Ctrl/GraphPlanner.cpp
	${GOLEM_ROOT}/src/Golem/Ctrl/Heuristic.cpp
	${GOLEM_ROOT}/src/Golem/Ctrl/Kinematics.cpp
	${GOLEM_ROOT}/src/Golem/Ctrl/PathFinder.cpp
	${GOLEM_ROOT}/src/Golem/Ctrl/Planner.cpp
	${GOLEM_ROOT}/src/Golem/Ctrl/Profile.cpp
	${GOLEM_ROOT}/src/Golem/Ctrl/ReacPlanner.cpp
	${GOLEM_ROOT}/src/Golem/Ctrl/Transmitter.cpp
)
SET(CTRL_HEADERS
	${GOLEM_ROOT}/include/Golem/Ctrl/Arm.h
	${GOLEM_ROOT}/include/Golem/Ctrl/Data.h
	${GOLEM_ROOT}/include/Golem/Ctrl/DEKinematics.h
	${GOLEM_ROOT}/include/Golem/Ctrl/GraphPlanner.h
	${GOLEM_ROOT}/include/Golem/Ctrl/Heuristic.h
	${GOLEM_ROOT}/include/Golem/Ctrl/Kinematics.h
	${GOLEM_ROOT}/include/Golem/Ctrl/LoadArmDesc.h
	${GOLEM_ROOT}/include/Golem/Ctrl/Msg.h
	${GOLEM_ROOT}/include/Golem/Ctrl/PathFinder.h
	${GOLEM_ROOT}/include/Golem/Ctrl/Planner.h
	${GOLEM_ROOT}/include/Golem/Ctrl/Profile.h
	${GOLEM_ROOT}/include/Golem/Ctrl/ReacPlanner.h
	${GOLEM_ROOT}/include/Golem/Ctrl/Transmitter.h
	${GOLEM_ROOT}/include/Golem/Ctrl/Waypoint.h
)
ADD_LIBRARY(GolemCtrl SHARED ${CTRL_SOURCES})
TARGET_LINK_LIBRARIES(GolemCtrl GolemDefs GolemMath GolemSys GolemTools)
INSTALL(TARGETS GolemCtrl LIBRARY DESTINATION lib)
INSTALL(FILES ${CTRL_HEADERS} DESTINATION include/Golem/Ctrl/)
endif(BUILD_CTRL)


###############################################################################
#
# Golem phys library
#
###############################################################################

if(BUILD_PHYS)
SET(PHYS_SOURCES
	${GOLEM_ROOT}/src/Golem/Phys/Application.cpp
	${GOLEM_ROOT}/src/Golem/Phys/Data.cpp
	${GOLEM_ROOT}/src/Golem/Phys/NxCooking.cpp
	${GOLEM_ROOT}/src/Golem/Phys/NxDebugRenderer.cpp
	${GOLEM_ROOT}/src/Golem/Phys/NxStream.cpp
	${GOLEM_ROOT}/src/Golem/Phys/Object.cpp
	${GOLEM_ROOT}/src/Golem/Phys/Recorder.cpp
	${GOLEM_ROOT}/src/Golem/Phys/Renderer.cpp
	${GOLEM_ROOT}/src/Golem/Phys/Scene.cpp
	${GOLEM_ROOT}/src/Golem/Phys/Universe.cpp
)
SET(PHYS_HEADERS
	${GOLEM_ROOT}/include/Golem/Phys/Application.h
	${GOLEM_ROOT}/include/Golem/Phys/Data.h
	${GOLEM_ROOT}/include/Golem/Phys/Msg.h
	${GOLEM_ROOT}/include/Golem/Phys/NxCooking.h
	${GOLEM_ROOT}/include/Golem/Phys/NxDebugRenderer.h
	${GOLEM_ROOT}/include/Golem/Phys/NxStream.h
	${GOLEM_ROOT}/include/Golem/Phys/Object.h
	${GOLEM_ROOT}/include/Golem/Phys/Recorder.h
	${GOLEM_ROOT}/include/Golem/Phys/Renderer.h
	${GOLEM_ROOT}/include/Golem/Phys/Scene.h
	${GOLEM_ROOT}/include/Golem/Phys/Universe.h
)
ADD_LIBRARY(GolemPhys SHARED ${PHYS_SOURCES})
TARGET_LINK_LIBRARIES(GolemPhys GolemDefs GolemMath GolemSys GolemTools NxCharacter NxCooking PhysXCore PhysXLoader GL GLU glut)
INSTALL(TARGETS GolemPhys LIBRARY DESTINATION lib)
INSTALL(FILES ${PHYS_HEADERS} DESTINATION include/Golem/Phys/)
endif(BUILD_PHYS)

###############################################################################
#
# Golem phys-ctrl library
#
###############################################################################

if(BUILD_PHYS_CTRL)
SET(PHYSCTRL_SOURCES
	${GOLEM_ROOT}/src/Golem/PhysCtrl/Creator.cpp
	${GOLEM_ROOT}/src/Golem/PhysCtrl/PhysArm.cpp
	${GOLEM_ROOT}/src/Golem/PhysCtrl/PhysPlanner.cpp
	${GOLEM_ROOT}/src/Golem/PhysCtrl/PhysReacPlanner.cpp
)
SET(PHYSCTRL_HEADERS
	${GOLEM_ROOT}/include/Golem/PhysCtrl/Creator.h
	${GOLEM_ROOT}/include/Golem/PhysCtrl/Msg.h
	${GOLEM_ROOT}/include/Golem/PhysCtrl/PhysArm.h
	${GOLEM_ROOT}/include/Golem/PhysCtrl/PhysPlanner.h
	${GOLEM_ROOT}/include/Golem/PhysCtrl/PhysReacPlanner.h
)
ADD_LIBRARY(GolemPhysCtrl SHARED ${PHYSCTRL_SOURCES})
TARGET_LINK_LIBRARIES(GolemPhysCtrl GolemPhys GolemCtrl)
INSTALL(TARGETS GolemPhysCtrl LIBRARY DESTINATION lib)
INSTALL(FILES ${PHYSCTRL_HEADERS} DESTINATION include/Golem/PhysCtrl/)
endif(BUILD_PHYS_CTRL)


###############################################################################
#
# Golem Device BufCtrl library
#
###############################################################################

if(BUILD_DEVICE_BUFCTRL)
SET(DEVICE_BUFCTRL_SOURCES
	${GOLEM_ROOT}/src/Golem/Device/BufCtrl/BufCtrl.cpp
	${GOLEM_ROOT}/src/Golem/Device/BufCtrl/Data.cpp
)
SET(DEVICE_BUFCTRL_HEADERS
	${GOLEM_ROOT}/include/Golem/Device/BufCtrl/BufCtrl.h
	${GOLEM_ROOT}/include/Golem/Device/BufCtrl/Data.h
)
ADD_LIBRARY(GolemDeviceBufCtrl STATIC ${DEVICE_BUFCTRL_SOURCES})
TARGET_LINK_LIBRARIES(GolemDeviceBufCtrl GolemDefs GolemMath GolemSys GolemTools GolemCtrl)
INSTALL(TARGETS GolemDeviceBufCtrl LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
INSTALL(FILES ${DEVICE_BUFCTRL_HEADERS} DESTINATION include/Golem/Device/BufCtrl)
endif(BUILD_DEVICE_BUFCTRL)


###############################################################################
#
# Golem Device BufCtrlSim library
#
###############################################################################

if(BUILD_DEVICE_BUFCTRLSIM)
SET(DEVICE_BUFCTRLSIM_SOURCES
	${GOLEM_ROOT}/src/Golem/Device/BufCtrlSim/BufCtrlSim.cpp
	${GOLEM_ROOT}/src/Golem/Device/BufCtrlSim/Data.cpp
)
SET(DEVICE_BUFCTRLSIM_HEADERS
	${GOLEM_ROOT}/include/Golem/Device/BufCtrlSim/BufCtrlSim.h
	${GOLEM_ROOT}/include/Golem/Device/BufCtrlSim/Data.h
)
ADD_LIBRARY(GolemDeviceBufCtrlSim STATIC ${DEVICE_BUFCTRLSIM_SOURCES})
TARGET_LINK_LIBRARIES(GolemDeviceBufCtrlSim GolemDeviceBufCtrl)
INSTALL(TARGETS GolemDeviceBufCtrlSim LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
INSTALL(FILES ${DEVICE_BUFCTRLSIM_HEADERS} DESTINATION include/Golem/Device/BufCtrlSim)
endif(BUILD_DEVICE_BUFCTRLSIM)


###############################################################################
#
# Golem Device Katana common library
#
###############################################################################

if(BUILD_DEVICE_KATANA)
SET(DEVICE_KATANA_SOURCES
	${GOLEM_ROOT}/src/Golem/Device/Katana/Katana.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana/Data.cpp
)
SET(DEVICE_KATANA_HEADERS
	${GOLEM_ROOT}/include/Golem/Device/Katana/Katana.h
	${GOLEM_ROOT}/include/Golem/Device/Katana/Data.h
)
ADD_LIBRARY(GolemDeviceKatana STATIC ${DEVICE_KATANA_SOURCES})
TARGET_LINK_LIBRARIES(GolemDeviceKatana GolemDefs GolemMath GolemSys GolemTools GolemCtrl)
INSTALL(TARGETS GolemDeviceKatana LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
INSTALL(FILES ${DEVICE_KATANA_HEADERS} DESTINATION include/Golem/Device/Katana)
endif(BUILD_DEVICE_KATANA)


###############################################################################
#
# Golem Device Katana300 library
#
###############################################################################

if(BUILD_DEVICE_KATANA300)
SET(DEVICE_KATANA300_SOURCES
	${GOLEM_ROOT}/src/Golem/Device/Katana300/Data.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana300/Katana300.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana300/kmlBase.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana300/kmlExt.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana300/kmlFactories.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana300/kmlMotBase.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana300/kmlSctBase.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana300/Timer.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana300/cdlCOM.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana300/cplSerial.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana300/CRC.cpp
)
SET(DEVICE_KATANA300_HEADERS
	${GOLEM_ROOT}/include/Golem/Device/Katana300/Katana300.h
	${GOLEM_ROOT}/include/Golem/Device/Katana300/Data.h
)
SET(DEVICE_KATANA300_FILES
	${GOLEM_ROOT}/resources/Device/GolemDeviceKatana300.xml
	${GOLEM_ROOT}/resources/Device/katana300_6M180.cfg
)
ADD_LIBRARY(GolemDeviceKatana300 SHARED ${DEVICE_KATANA300_SOURCES})
TARGET_LINK_LIBRARIES(GolemDeviceKatana300 GolemDeviceBufCtrl GolemDeviceKatana)
INSTALL(TARGETS GolemDeviceKatana300 LIBRARY DESTINATION lib)
INSTALL(FILES ${DEVICE_KATANA300_HEADERS} DESTINATION include/Golem/Device/Katana300)
INSTALL(FILES ${DEVICE_KATANA300_FILES} DESTINATION bin/Golem)
INSTALL(FILES ${DEVICE_KATANA300_FILES} DESTINATION bin/Golem/Demo)
endif(BUILD_DEVICE_KATANA300)


###############################################################################
#
# Golem Device Katana300Sim library
#
###############################################################################

if(BUILD_DEVICE_KATANA300SIM)
SET(DEVICE_KATANA300SIM_SOURCES
	${GOLEM_ROOT}/src/Golem/Device/Katana300Sim/Katana300Sim.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana300Sim/Data.cpp
)
SET(DEVICE_KATANA300SIM_HEADERS
	${GOLEM_ROOT}/include/Golem/Device/Katana300Sim/Katana300Sim.h
	${GOLEM_ROOT}/include/Golem/Device/Katana300Sim/Data.h
)
SET(DEVICE_KATANA300SIM_FILES
	${GOLEM_ROOT}/resources/Device/GolemDeviceKatana300Sim.xml
)
ADD_LIBRARY(GolemDeviceKatana300Sim SHARED ${DEVICE_KATANA300SIM_SOURCES})
TARGET_LINK_LIBRARIES(GolemDeviceKatana300Sim GolemDeviceBufCtrlSim GolemDeviceKatana)
INSTALL(TARGETS GolemDeviceKatana300Sim LIBRARY DESTINATION lib)
INSTALL(FILES ${DEVICE_KATANA300SIM_HEADERS} DESTINATION include/Golem/Device/Katana300Sim)
INSTALL(FILES ${DEVICE_KATANA300SIM_FILES} DESTINATION bin/Golem)
INSTALL(FILES ${DEVICE_KATANA300SIM_FILES} DESTINATION bin/Golem/Demo)
endif(BUILD_DEVICE_KATANA300SIM)


###############################################################################
#
# Golem Device Katana450 library
#
###############################################################################

if(BUILD_DEVICE_KATANA450)
SET(DEVICE_KATANA450_SOURCES
	${GOLEM_ROOT}/src/Golem/Device/Katana450/Data.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana450/Katana450.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana450/AxNI_base.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana450/axis_controller_emcy.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana450/can_bootloader_master.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana450/canbus_interface.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana450/canopen_emcy_queue.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana450/canopen_master.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana450/canopen_master_node.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana450/canopen_nmt_master.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana450/pdo_interface.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana450/Timer.cpp
	${GOLEM_ROOT}/src/Golem/Device/Katana450/tms320f2808dsp.cpp
)
SET(DEVICE_KATANA450_HEADERS
	${GOLEM_ROOT}/include/Golem/Device/Katana450/Data.h
	${GOLEM_ROOT}/include/Golem/Device/Katana450/Katana450.h
	${GOLEM_ROOT}/include/Golem/Device/Katana450/AxNI_base.h
	${GOLEM_ROOT}/include/Golem/Device/Katana450/axis_config.h
)
SET(DEVICE_KATANA450_FILES
	${GOLEM_ROOT}/resources/Device/GolemDeviceKatana450.xml
)
ADD_LIBRARY(GolemDeviceKatana450 SHARED ${DEVICE_KATANA450_SOURCES})
TARGET_LINK_LIBRARIES(GolemDeviceKatana450 GolemDeviceBufCtrl GolemDeviceKatana pcan ${Boost_LIBRARIES})
INSTALL(TARGETS GolemDeviceKatana450 LIBRARY DESTINATION lib)
INSTALL(FILES ${DEVICE_KATANA450_HEADERS} DESTINATION include/Golem/Device/Katana450)
INSTALL(FILES ${DEVICE_KATANA450_FILES} DESTINATION bin/Golem)
INSTALL(FILES ${DEVICE_KATANA450_FILES} DESTINATION bin/Golem/Demo)
endif(BUILD_DEVICE_KATANA450)


###############################################################################
#
# Golem Device SixAxisSim library
#
###############################################################################

if(BUILD_DEVICE_SIXAXISSIM)
SET(DEVICE_SIXAXISSIM_SOURCES
	${GOLEM_ROOT}/src/Golem/Device/SixAxisSim/SixAxisSim.cpp
	${GOLEM_ROOT}/src/Golem/Device/SixAxisSim/Data.cpp
)
SET(DEVICE_SIXAXISSIM_HEADERS
	${GOLEM_ROOT}/include/Golem/Device/SixAxisSim/SixAxisSim.h
	${GOLEM_ROOT}/include/Golem/Device/SixAxisSim/Data.h
)
SET(DEVICE_SIXAXISSIM_FILES
	${GOLEM_ROOT}/resources/Device/GolemDeviceSixAxisSim.xml
)
ADD_LIBRARY(GolemDeviceSixAxisSim SHARED ${DEVICE_SIXAXISSIM_SOURCES})
TARGET_LINK_LIBRARIES(GolemDeviceSixAxisSim GolemDeviceBufCtrlSim)
INSTALL(TARGETS GolemDeviceSixAxisSim LIBRARY DESTINATION lib)
INSTALL(FILES ${DEVICE_SIXAXISSIM_HEADERS} DESTINATION include/Golem/Device/SixAxisSim)
INSTALL(FILES ${DEVICE_SIXAXISSIM_FILES} DESTINATION bin/Golem)
INSTALL(FILES ${DEVICE_SIXAXISSIM_FILES} DESTINATION bin/Golem/Demo)
endif(BUILD_DEVICE_SIXAXISSIM)


###############################################################################
#
# Golem demo library
#
###############################################################################

if(BUILD_DEMO_COMMON)
SET(DEMOCOMMON_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/Common/Data.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Common/DigitalFilter.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Common/Embodiment.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Common/Finger.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Common/FTSensor.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Common/Image.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Common/ImageFilter.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Common/Layer.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Common/PointTracker.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Common/Recognition.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Common/Retina.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Common/RigidBodyTracker.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Common/Rotations.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Common/Tools.cpp
)
SET(DEMOCOMMON_HEADERS
	${GOLEM_ROOT}/include/Golem/Demo/Common/ArrN.h
	${GOLEM_ROOT}/include/Golem/Demo/Common/Data.h
	${GOLEM_ROOT}/include/Golem/Demo/Common/DigitalFilter.h
	${GOLEM_ROOT}/include/Golem/Demo/Common/Embodiment.h
	${GOLEM_ROOT}/include/Golem/Demo/Common/Finger.h
	${GOLEM_ROOT}/include/Golem/Demo/Common/FTSensor.h
	${GOLEM_ROOT}/include/Golem/Demo/Common/Image.h
	${GOLEM_ROOT}/include/Golem/Demo/Common/ImageFilter.h
	${GOLEM_ROOT}/include/Golem/Demo/Common/Layer.h
	${GOLEM_ROOT}/include/Golem/Demo/Common/Msg.h
	${GOLEM_ROOT}/include/Golem/Demo/Common/PointTracker.h
	${GOLEM_ROOT}/include/Golem/Demo/Common/Recognition.h
	${GOLEM_ROOT}/include/Golem/Demo/Common/Retina.h
	${GOLEM_ROOT}/include/Golem/Demo/Common/RigidBodyTracker.h
	${GOLEM_ROOT}/include/Golem/Demo/Common/Rotations.h
	${GOLEM_ROOT}/include/Golem/Demo/Common/Tools.h
)
ADD_LIBRARY(GolemDemoCommon SHARED ${DEMOCOMMON_SOURCES})
TARGET_LINK_LIBRARIES(GolemDemoCommon GolemPhysCtrl)
INSTALL(TARGETS GolemDemoCommon LIBRARY DESTINATION lib)
INSTALL(FILES ${DEMOCOMMON_HEADERS} DESTINATION include/Golem/Demo/Common/)
endif(BUILD_DEMO_COMMON)

###############################################################################
#
# Golem demo executables
#
###############################################################################

if(BUILD_AXNI)
# AxNI
SET(AXNI_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/AxNI/AxNI.cpp
)
SET(AXNI_FILES
	${GOLEM_ROOT}/resources/Demo/AxNI.xml
)
ADD_EXECUTABLE(AxNI ${AXNI_SOURCES})
#SET_TARGET_PROPERTIES(AxNI PROPERTIES LINK_FLAGS "-rdynamic -ldl")
TARGET_LINK_LIBRARIES(AxNI GolemDemoCommon GolemDeviceKatana450)
INSTALL(TARGETS AxNI RUNTIME DESTINATION bin/Golem/Demo)
INSTALL(FILES ${AXNI_FILES} DESTINATION bin/Golem/Demo)
endif(BUILD_AXNI)

if(BUILD_DEMO)
# Bounds
SET(BOUNDS_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/Bounds/Bounds.cpp
)
SET(BOUNDS_FILES
	${GOLEM_ROOT}/resources/Demo/Bounds.xml
)
ADD_EXECUTABLE(Bounds ${BOUNDS_SOURCES})
TARGET_LINK_LIBRARIES(Bounds GolemDemoCommon)
INSTALL(TARGETS Bounds RUNTIME DESTINATION bin/Golem/Demo)
INSTALL(FILES ${BOUNDS_FILES} DESTINATION bin/Golem/Demo)

# Collision
SET(COLLISION_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/Collision/Collision.cpp
)
ADD_EXECUTABLE(Collision ${COLLISION_SOURCES})
TARGET_LINK_LIBRARIES(Collision GolemDefs GolemMath GolemSys GolemTools GolemPhys)
INSTALL(TARGETS Collision RUNTIME DESTINATION bin/Golem/Demo)


# Finger
SET(FINGER_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/Finger/Robot.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Finger/Finger.cpp
)
SET(FINGER_FILES
	${GOLEM_ROOT}/resources/Demo/Finger.xml
)
ADD_EXECUTABLE(Finger ${FINGER_SOURCES})
TARGET_LINK_LIBRARIES(Finger GolemDemoCommon)
INSTALL(TARGETS Finger RUNTIME DESTINATION bin/Golem/Demo)
INSTALL(FILES ${FINGER_FILES} DESTINATION bin/Golem/Demo)

	
# Gripper
SET(GRIPPER_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/Gripper/Gripper.cpp
)
SET(GRIPPER_FILES
	${GOLEM_ROOT}/resources/Demo/Gripper.xml
)
ADD_EXECUTABLE(Gripper ${GRIPPER_SOURCES})
TARGET_LINK_LIBRARIES(Gripper GolemDemoCommon GolemDeviceKatana)
INSTALL(TARGETS Gripper RUNTIME DESTINATION bin/Golem/Demo)
INSTALL(FILES ${GRIPPER_FILES} DESTINATION bin/Golem/Demo)


# Joint
SET(JOINT_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/Joint/Joint.cpp
)
SET(JOINT_FILES
	${GOLEM_ROOT}/resources/Demo/Joint.xml
)
ADD_EXECUTABLE(Joint ${JOINT_SOURCES})
TARGET_LINK_LIBRARIES(Joint GolemDemoCommon)
INSTALL(TARGETS Joint RUNTIME DESTINATION bin/Golem/Demo)
INSTALL(FILES ${JOINT_FILES} DESTINATION bin/Golem/Demo)


# Movement
SET(MOVEMENT_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/Movement/Movement.cpp
)
SET(MOVEMENT_FILES
	${GOLEM_ROOT}/resources/Demo/Movement.xml
)
ADD_EXECUTABLE(Movement ${MOVEMENT_SOURCES})
TARGET_LINK_LIBRARIES(Movement GolemDemoCommon)
INSTALL(TARGETS Movement RUNTIME DESTINATION bin/Golem/Demo)
INSTALL(FILES ${MOVEMENT_FILES} DESTINATION bin/Golem/Demo)


# PhysJoint
SET(PHYSJOINT_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/PhysJoint/PhysJoint.cpp
)
SET(PHYSJOINT_FILES
	${GOLEM_ROOT}/resources/Demo/PhysJoint.xml
)
ADD_EXECUTABLE(PhysJoint ${PHYSJOINT_SOURCES})
TARGET_LINK_LIBRARIES(PhysJoint GolemDemoCommon)
INSTALL(TARGETS PhysJoint RUNTIME DESTINATION bin/Golem/Demo)
INSTALL(FILES ${PHYSJOINT_FILES} DESTINATION bin/Golem/Demo)


# Planner
SET(PLANNER_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/Planner/Planner.cpp
)
SET(PLANNER_FILES
	${GOLEM_ROOT}/resources/Demo/Planner.xml
)
ADD_EXECUTABLE(Planner ${PLANNER_SOURCES})
TARGET_LINK_LIBRARIES(Planner GolemDemoCommon)
INSTALL(TARGETS Planner RUNTIME DESTINATION bin/Golem/Demo)
INSTALL(FILES ${PLANNER_FILES} DESTINATION bin/Golem/Demo)


# Prediction
SET(PREDICTION_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/Prediction/Prediction.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Prediction/Predictor.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Prediction/Robot.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Prediction/Scenario.cpp
)
SET(PREDICTION_FILES
	${GOLEM_ROOT}/resources/Demo/Prediction.xml
)
ADD_EXECUTABLE(Prediction ${PREDICTION_SOURCES})
TARGET_LINK_LIBRARIES(Prediction GolemDemoCommon)
INSTALL(TARGETS Prediction RUNTIME DESTINATION bin/Golem/Demo)
INSTALL(FILES ${PREDICTION_FILES} DESTINATION bin/Golem/Demo)


# ReacPlanner
SET(REACPLANNER_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/ReacPlanner/ReacPlanner.cpp
)
SET(REACPLANNER_FILES
	${GOLEM_ROOT}/resources/Demo/ReacPlanner.xml
)
ADD_EXECUTABLE(ReacPlanner ${REACPLANNER_SOURCES})
TARGET_LINK_LIBRARIES(ReacPlanner GolemDemoCommon)
INSTALL(TARGETS ReacPlanner RUNTIME DESTINATION bin/Golem/Demo)
INSTALL(FILES ${REACPLANNER_FILES} DESTINATION bin/Golem/Demo)


# Recognition
SET(RECOGNITION_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/Recognition/Recognition.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Recognition/Robot.cpp
)
SET(RECOGNITION_FILES
	${GOLEM_ROOT}/resources/Demo/Recognition.xml
)
ADD_EXECUTABLE(Recognition ${RECOGNITION_SOURCES})
TARGET_LINK_LIBRARIES(Recognition GolemDemoCommon)
INSTALL(TARGETS Recognition RUNTIME DESTINATION bin/Golem/Demo)
INSTALL(FILES ${RECOGNITION_FILES} DESTINATION bin/Golem/Demo)


# Retina
SET(RETINA_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/Retina/Retina.cpp
	${GOLEM_ROOT}/src/Golem/Demo/Retina/Robot.cpp
)
SET(RETINA_FILES
	${GOLEM_ROOT}/resources/Demo/Retina.xml
)
ADD_EXECUTABLE(Retina ${RETINA_SOURCES})
TARGET_LINK_LIBRARIES(Retina GolemDemoCommon)
INSTALL(TARGETS Retina RUNTIME DESTINATION bin/Golem/Demo)
INSTALL(FILES ${RETINA_FILES} DESTINATION bin/Golem/Demo)


# Scenes
SET(SCENES_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/Scenes/Scenes.cpp
)
SET(SCENES_FILES
	${GOLEM_ROOT}/resources/Demo/Scenes.xml
)
ADD_EXECUTABLE(Scenes ${SCENES_SOURCES})
TARGET_LINK_LIBRARIES(Scenes GolemDemoCommon GolemDeviceKatana300Sim GolemDeviceSixAxisSim)
INSTALL(TARGETS Scenes RUNTIME DESTINATION bin/Golem/Demo)
INSTALL(FILES ${SCENES_FILES} DESTINATION bin/Golem/Demo)
endif(BUILD_DEMO)


###############################################################################
#
# Golem Tiny library and examples
#
###############################################################################

if(BUILD_TINY)
SET(GOLEM_TINY_SOURCES
	${GOLEM_ROOT}/src/Golem/Tiny/Tiny.cpp
)
ADD_LIBRARY(GolemTiny SHARED ${GOLEM_TINY_SOURCES})


if(BUILD_DEVICE_KATANA300 AND BUILD_DEVICE_KATANA450)
TARGET_LINK_LIBRARIES(GolemTiny GolemPhysCtrl GolemDeviceKatana300Sim GolemDeviceKatana300 GolemDeviceKatana450 GolemDeviceSixAxisSim)
elseif(BUILD_DEVICE_KATANA300)
TARGET_LINK_LIBRARIES(GolemTiny GolemPhysCtrl GolemDeviceKatana300Sim GolemDeviceKatana300 GolemDeviceSixAxisSim)
elseif(BUILD_DEVICE_KATANA450)
TARGET_LINK_LIBRARIES(GolemTiny GolemPhysCtrl GolemDeviceKatana300Sim GolemDeviceKatana450 GolemDeviceSixAxisSim)
endif(BUILD_DEVICE_KATANA300 AND BUILD_DEVICE_KATANA450)

INSTALL(TARGETS GolemTiny LIBRARY DESTINATION lib)

SET(TINY_HEADERS
	${GOLEM_ROOT}/include/Golem/Tiny/Types.h
	${GOLEM_ROOT}/include/Golem/Tiny/Tiny.h
)
INSTALL(FILES ${TINY_HEADERS} DESTINATION include/Golem/Tiny/)
endif(BUILD_TINY)

if(BUILD_TINY_DEMO)
# TinyPushing
SET(TINYPUSHING_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/TinyPushing/TinyPushing.cpp
)
SET(TINYPUSHING_FILES
	${GOLEM_ROOT}/resources/Demo/TinyPushing.xml
)
ADD_EXECUTABLE(TinyPushing ${TINYPUSHING_SOURCES})
TARGET_LINK_LIBRARIES(TinyPushing GolemTiny)
INSTALL(TARGETS TinyPushing RUNTIME DESTINATION bin/Golem/Demo)
INSTALL(FILES ${TINYPUSHING_FILES} DESTINATION bin/Golem/Demo)

# TinyGrasp
SET(TINYGRASP_HEADERS
	${GOLEM_ROOT}/include/Golem/Demo/TinyGrasp/TinyGrasp.h
)
SET(TINYGRASP_SOURCES
	${GOLEM_ROOT}/src/Golem/Demo/TinyGrasp/main.cpp
	${GOLEM_ROOT}/src/Golem/Demo/TinyGrasp/TinyGrasp.cpp
)
SET(TINYGRASP_FILES
	${GOLEM_ROOT}/resources/Demo/TinyGrasp.xml
)
ADD_EXECUTABLE(TinyGrasp ${TINYGRASP_SOURCES})
TARGET_LINK_LIBRARIES(TinyGrasp GolemTiny)
INSTALL(TARGETS TinyGrasp RUNTIME DESTINATION bin/Golem/Demo)
INSTALL(FILES ${TINYGRASP_FILES} DESTINATION bin/Golem/Demo)
endif(BUILD_TINY_DEMO)

###############################################################################
#
# Golem Tiny ZeroC Ice server and examples
#
###############################################################################

if(BUILD_TINY_ICE)
# TinyIce
SET(TINYICE_HEADERS
	${GOLEM_ROOT}/include/Golem/TinyIce/Desc.h
	${GOLEM_ROOT}/include/Golem/TinyIce/Types.h
	${GOLEM_ROOT}/include/Golem/TinyIce/TinyIceI.h
)
SET(TINYICE_RESOURCE_HEADER
	${GOLEM_ROOT}/resources/TinyIce/TinyIce.hh
)
SET(TINYICE_SOURCES
	${GOLEM_ROOT}/resources/TinyIce/TinyIce.cc
	${GOLEM_ROOT}/src/Golem/TinyIce/TinyIceI.cpp
)
SET(TINYICE_FILES
	${GOLEM_ROOT}/resources/TinyIce/TinyIce.xml
)
ADD_CUSTOM_COMMAND(
	OUTPUT ${GOLEM_ROOT}/resources/TinyIce/TinyIce.cc ${GOLEM_ROOT}/resources/TinyIce/TinyIce.hh
	COMMAND ${ICE_BIN}/slice2cpp
	ARGS --header-ext hh --source-ext cc --output-dir ${GOLEM_ROOT}/resources/TinyIce --include-dir TinyIce ${GOLEM_ROOT}/resources/TinyIce/TinyIce.ice
	DEPENDS ${GOLEM_ROOT}/resources/TinyIce/TinyIce.ice
	COMMENT "slice2cpp"
)

ADD_EXECUTABLE(TinyIce ${TINYICE_SOURCES})


if(BUILD_DEVICE_KATANA300 AND BUILD_DEVICE_KATANA450)
TARGET_LINK_LIBRARIES(TinyIce GolemPhysCtrl GolemDeviceKatana300Sim GolemDeviceKatana300 GolemDeviceKatana450 GolemDeviceSixAxisSim Ice IceUtil)
elseif(BUILD_DEVICE_KATANA300)
TARGET_LINK_LIBRARIES(TinyIce GolemPhysCtrl GolemDeviceKatana300Sim GolemDeviceKatana300 GolemDeviceSixAxisSim Ice IceUtil)
elseif(BUILD_DEVICE_KATANA450)
TARGET_LINK_LIBRARIES(TinyIce GolemPhysCtrl GolemDeviceKatana300Sim GolemDeviceKatana450 GolemDeviceSixAxisSim Ice IceUtil)
endif(BUILD_DEVICE_KATANA300 AND BUILD_DEVICE_KATANA450)


INSTALL(FILES ${TINYICE_HEADERS} DESTINATION include/Golem/TinyIce/)
INSTALL(FILES ${TINYICE_RESOURCE_HEADER} DESTINATION include/Golem/TinyIce/)
INSTALL(TARGETS TinyIce RUNTIME DESTINATION bin/Golem)
INSTALL(FILES ${TINYICE_FILES} DESTINATION bin/Golem)
INSTALL(FILES ${GOLEM_ROOT}/resources/TinyIce/TinyIce.ice DESTINATION share/golem/slice)
endif(BUILD_TINY_ICE)


if(BUILD_TINY_ICE_DEMO)
# TinyIcePushing
SET(TINYICEPUSHING_SOURCES
	${GOLEM_ROOT}/resources/TinyIce/TinyIce.cc
	${GOLEM_ROOT}/src/Golem/Demo/TinyIcePushing/TinyIcePushing.cpp
)
ADD_EXECUTABLE(TinyIcePushing ${TINYICEPUSHING_SOURCES})
TARGET_LINK_LIBRARIES(TinyIcePushing Ice IceUtil)
INSTALL(TARGETS TinyIcePushing RUNTIME DESTINATION bin/Golem/Demo)
endif(BUILD_TINY_ICE_DEMO)

