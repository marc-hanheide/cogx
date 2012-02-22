cmake_minimum_required(VERSION 2.4)

# add_subdirectory (src/c++/categorical/autogen)
if (DO_SLICE_ONLY)
   return()
endif()

set(Boost_USE_MULTITHREADED ON)
FIND_PACKAGE (Boost 1.38.0 REQUIRED COMPONENTS date_time thread program_options serialization regex filesystem system signals)
FIND_PACKAGE (OpenCV REQUIRED)
INCLUDE_DIRECTORIES (${OPENCV_INCLUDE_DIRS})

add_definitions(-DBOOST_FILESYSTEM_VERSION=2)

option(BUILD_CALIB_CAM "Build single monocular camera calibration" YES)
if(BUILD_CALIB_CAM)
  add_executable(camcalib camcalib.cpp)
  target_link_libraries(camcalib ${OpenCV_LIBS} ${Boost_LIBRARIES} )
  INSTALL(TARGETS camcalib RUNTIME DESTINATION bin)
endif(BUILD_CALIB_CAM)

option(BUILD_CALIB_PTU "Build PTU calibration" YES)
if(BUILD_CALIB_PTU)
  add_executable(ptucalib_simple ptucalib_simple.cpp)
  target_link_libraries(ptucalib_simple ${OpenCV_LIBS})
  INSTALL(TARGETS ptucalib_simple RUNTIME DESTINATION bin)
endif(BUILD_CALIB_PTU)

option(BUILD_CALIB_ARM "Build robot arm calibration" NO)
if(BUILD_CALIB_ARM)
  # SET(GOLEM_ROOT "" CACHE PATH "Path to golem installation directory")
  SET(PHYSX_INCLUDE /usr/include/PhysX/v2.8.3 CACHE PATH "Path prefix for PhysX include")
  SET(PHYSX_LIBRARY /usr/lib/PhysX/v2.8.3 CACHE PATH "Path prefix for PhysX library")

  INCLUDE_DIRECTORIES(
    # ${GOLEM_ROOT}/include
    # ${GOLEM_ROOT}/resources
    # ${GOLEM_ROOT}/include/Golem/Demo
    ${PHYSX_INCLUDE}/SDKs/Physics/include
    ${PHYSX_INCLUDE}/SDKs/Foundation/include
    ${PHYSX_INCLUDE}/SDKs/PhysXLoader/include
    ${PHYSX_INCLUDE}/SDKs/Cooking/include
  )

  LINK_DIRECTORIES(
    ${PHYSX_LIBRARY}
  )

  SET(CMAKE_CXX_FLAGS ${CXX_FLAGS} "-O2 -fPIC -DLINUX -DNX64")

  ADD_EXECUTABLE(armcalib armcalib.cpp)
  TARGET_LINK_LIBRARIES(armcalib ${OPENCV_LDFLAGS} ${Boost_LIBRARIES} GolemDemoCommon GolemTiny)
  INSTALL(TARGETS armcalib RUNTIME DESTINATION bin)

  # SET(ARMCALIB_ICE_SOURCES
  #   TinyIce.cc
  #   armcalib_ice.cpp
  # )
  # ADD_EXECUTABLE(armcalib_ice ${ARMCALIB_ICE_SOURCES})
  # TARGET_LINK_LIBRARIES(armcalib_ice Ice IceUtil)
  # INSTALL(TARGETS armcalib_ice RUNTIME DESTINATION bin)
endif(BUILD_CALIB_ARM)

