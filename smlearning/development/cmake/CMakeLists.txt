cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


# SMLearning
PROJECT(SMLearning)


# Project root directory
GET_FILENAME_COMPONENT(PROJECT_ROOT ../.. ABSOLUTE)
SET(SMLEARNING_ROOT ${PROJECT_ROOT}/development CACHE INTERNAL "Path prefix for SMLearning package")

# Set the path where other thing should be relative to
GET_FILENAME_COMPONENT(CMAKE_SOURCE_DIR . ABSOLUTE CACHE INTERNAL "")

# Single output directory for building all SMLearning libraries
SET(LIBRARY_OUTPUT_PATH ${SMLEARNING_ROOT}/lib/linux CACHE PATH "Single output directory for building all SMLearning libraries")

# Single output directory for building all SMLearning executables
SET(EXECUTABLE_OUTPUT_PATH ${SMLEARNING_ROOT}/bin/linux CACHE PATH "Single output directory for building all SMLearning executables")

# Set the install directory.
SET(CMAKE_INSTALL_PREFIX /usr/local CACHE PATH "Path prefix for SMLearning installation")

SET(PHYSX_INCLUDE /usr/include/PhysX/v2.8.1 CACHE PATH "Path prefix for PhysX include")
SET(PHYSX_LIBRARY /usr/lib/PhysX/v2.8.1 CACHE PATH "Path prefix for PhysX library")

SET(KNI_INCLUDE /usr/local/include/KNI CACHE PATH "Path prefix for KNI include")
SET(KNI_LIBRARY /usr/local/lib CACHE PATH "Path prefix for KNI library")

SET(GOLEM_INCLUDE /usr/local/include/Golem CACHE PATH "Path prefix for Golem include")
SET(GOLEM_LIBRARY /usr/local/lib CACHE PATH "Path prefix for Golem library")

SET(SMLEARNING_INCLUDE ${SMLEARNING_ROOT}/include CACHE PATH "Path prefix for SMLearning include")

ADD_DEFINITIONS(-DLINUX -fpermissive -DNX_DISABLE_FLUIDS)
LINK_LIBRARIES(NxCharacter NxCooking PhysXCore PhysXLoader KNI Golem pthread rt GL glut expat SMLearning)

INCLUDE_DIRECTORIES(
	${SMLEARNING_INCLUDE}
	${GOLEM_INCLUDE}
	${GOLEM_INCLUDE}/demo/common
	${PHYSX_INCLUDE}/SDKs/Physics/include
	${PHYSX_INCLUDE}/SDKs/Foundation/include
	${PHYSX_INCLUDE}/SDKs/PhysXLoader/include
	${PHYSX_INCLUDE}/SDKs/Cooking/include
	${KNI_INCLUDE}
)

LINK_DIRECTORIES(
	${PHYSX_LIBRARY}
	${KNI_LIBRARY}
	${GOLEM_LIBRARY}
)

SET(SMLEARNING_SOURCES
	${SMLEARNING_ROOT}/src/arm/Pushing.cpp
)

SET(ARM_HEADERS
#	${SMLEARNING_ROOT}/include/arm/Pushing.h
)
#INSTALL(FILES ${ARM_HEADERS} DESTINATION include/SMLearning/arm/)

SET(ARM_FILES
	${SMLEARNING_ROOT}/src/arm.xml
)

SET(TOOLS_SOURCES
	${SMLEARNING_ROOT}/src/tools/data_handling.cpp
)

SET(TOOLS_HEADERS
	${SMLEARNING_ROOT}/include/tools/data_handling.h
)
INSTALL(FILES ${TOOLS_HEADERS} DESTINATION include/SMLearning/tools/)

# SMLearning library
# ADD_LIBRARY(${PROJECT_NAME} SHARED ${SMLEARNING_SOURCES} ${TOOLS_SOURCES})
ADD_LIBRARY(${PROJECT_NAME} SHARED ${TOOLS_SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

ADD_EXECUTABLE(arm ${SMLEARNING_SOURCES})
ADD_EXECUTABLE(tools_test ${TOOLS_SOURCES})
INSTALL(TARGETS arm RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS tools_test RUNTIME DESTINATION bin/SMLearning)
INSTALL(FILES ${ARM_FILES} DESTINATION bin/SMLearning)


## SMLearning tools library
#ADD_EXECUTABLE(tools_test ${TOOLS_SOURCES})
#INSTALL(TARGETS tools_test RUNTIME DESTINATION bin/SMLearning)
#ADD_LIBRARY(${PROJECT_NAME}_tools SHARED ${TOOLS_SOURCES})
#INSTALL(TARGETS ${PROJECT_NAME}_tools LIBRARY DESTINATION lib)

