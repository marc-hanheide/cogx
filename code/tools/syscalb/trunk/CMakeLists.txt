cmake_minimum_required(VERSION 2.4)

set(Boost_USE_MULTITHREADED ON)
FIND_PACKAGE (Boost 1.38.0 REQUIRED COMPONENTS date_time thread program_options serialization regex filesystem signals)
FIND_PACKAGE (OpenCV REQUIRED)


option(BUILD_CAMCALIB "Build single monocular camera calibration" YES)
if(BUILD_CAMCALIB)
  add_executable(camcalib camcalib.cpp)
  target_link_libraries(camcalib ${OpenCV_LIBS} ${Boost_LIBRARIES} )
  INSTALL(TARGETS camcalib RUNTIME DESTINATION bin)
endif(BUILD_CAMCALIB)

option(BUILD_PTUCALIB "Build PTU calibration" YES)
if(BUILD_PTUCALIB)
  add_executable(ptucalib_simple ptucalib_simple.cpp)
  target_link_libraries(ptucalib_simple ${OpenCV_LIBS} )
  INSTALL(TARGETS ptucalib_simple RUNTIME DESTINATION bin)
endif(BUILD_PTUCALIB)

option(BUILD_ARMCALIB "Build robot arm calibration" NO)
if(BUILD_ARMCALIB)

  SET(PHYSX_INCLUDE /usr/include/PhysX/v2.8.3 CACHE PATH "Path prefix for PhysX include")
  SET(PHYSX_LIBRARY /usr/lib/PhysX/v2.8.3 CACHE PATH "Path prefix for PhysX library")

  INCLUDE_DIRECTORIES(
#	${GOLEM_ROOT}/include
#	${GOLEM_ROOT}/resources
#	${GOLEM_ROOT}/include/Golem/Demo
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
  TARGET_LINK_LIBRARIES(armcalib ${OpenCV_LIBS} GolemDemoCommon GolemTiny)
  INSTALL(TARGETS armcalib RUNTIME DESTINATION bin)
endif(BUILD_ARMCALIB)


