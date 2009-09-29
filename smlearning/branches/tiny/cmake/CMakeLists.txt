cmake_minimum_required(VERSION 2.6)
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


# SMLearning
PROJECT(SMLearning)


# Project root directory
GET_FILENAME_COMPONENT(PROJECT_ROOT ../.. ABSOLUTE)
SET(SMLEARNING_ROOT ${PROJECT_ROOT}/tiny CACHE INTERNAL "Path prefix for SMLearning package")

# Look for Boost libraries
SET(CMAKE_MODULE_PATH ${SMLEARNING_ROOT}/cmake)
INCLUDE(${SMLEARNING_ROOT}/cmake/FindBoost.cmake)


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

SET(RNNLIB_INCLUDE /usr/local/include/rnnlib CACHE PATH "Path prefix for RNNLIB include")
SET(RNNLIB_LIBRARY /usr/local/lib CACHE PATH "Path prefix for RNNLIB library")

SET(Boost_MAJOR_VERSION 1)
SET(Boost_MINOR_VERSION 36)
# SET(Boost_SUBMINOR_VERSION 0)
FIND_PACKAGE( Boost 1.36 COMPONENTS filesystem regex )

# If boost was not found:
# Use the following values if you use Ubuntu 8.10 and boost 1.39.0
# SET(Boost_INCLUDE_DIRS /usr/local/include/boost-1_39 CACHE PATH "Path prefix for Golem include")
# SET(Boost_LIBRARY_DIRS /usr/local/lib CACHE PATH "Path prefix for Boost library")
# SET(Boost_FILESYSTEM_LIBRARY
#	/usr/local/lib/libboost_filesystem-gcc43-mt-1_39.so.1.39.0 CACHE PATH
#	"Path to Boost Filesystem library")
# SET(Boost_REGEX_LIBRARY
#	/usr/local/lib/libboost_regex-gcc43-mt-1_39.so.1.39.0 CACHE PATH
# 	"Path to Boost Regex Library")

SET(SMLEARNING_INCLUDE ${SMLEARNING_ROOT}/include CACHE PATH "Path prefix for SMLearning include")

ADD_DEFINITIONS(-DLINUX -fpermissive -DNX_DISABLE_FLUIDS)
LINK_LIBRARIES(NxCharacter NxCooking PhysXCore PhysXLoader KNI Golem GolemTinyIce
			   pthread rt GL glut expat SMLearning netcdf
 			   rnnlib ${Boost_FILESYSTEM_LIBRARY} ${Boost_REGEX_LIBRARY})

INCLUDE_DIRECTORIES(
	${SMLEARNING_INCLUDE}
	${GOLEM_INCLUDE}
	${GOLEM_INCLUDE}/demo/common
	${PHYSX_INCLUDE}/SDKs/Physics/include
	${PHYSX_INCLUDE}/SDKs/Foundation/include
	${PHYSX_INCLUDE}/SDKs/PhysXLoader/include
	${PHYSX_INCLUDE}/SDKs/Cooking/include
	${KNI_INCLUDE}
	${Boost_INCLUDE_DIRS}
	${RNNLIB_INCLUDE}
)

LINK_DIRECTORIES(
	${PHYSX_LIBRARY}
	${KNI_LIBRARY}
	${GOLEM_LIBRARY}
	${Boost_LIBRARY_DIRS}
	${RNNLIB_LIBRARY}
)

SET(METALEARNING_SOURCES
	${SMLEARNING_ROOT}/src/metalearning/Scenario.cpp
	${SMLEARNING_ROOT}/src/metalearning/OfflineRNN.cpp
	${SMLEARNING_ROOT}/src/metalearning/ScenarioIce.cpp
)

SET(OFFLINE_EXPERIMENT_SOURCE
	${SMLEARNING_ROOT}/src/metalearning/offline_experiment.cpp
)

SET(OFFLINE_EXPERIMENT_ICE_SOURCE
	${SMLEARNING_ROOT}/src/metalearning/offline_experiment_ice.cpp
)

SET(ENSEMBLE_OFF_EXPERIMENT_SOURCE
	${SMLEARNING_ROOT}/src/metalearning/ensemble_off_experiment.cpp
)

SET(METALEARNING_HEADERS
	${SMLEARNING_ROOT}/include/metalearning/Scenario.h
	${SMLEARNING_ROOT}/include/metalearning/OfflineRNN.h
	${SMLEARNING_ROOT}/include/metalearning/ScenarioIce.h
)
INSTALL(FILES ${METALEARNING_HEADERS} DESTINATION include/SMLearning/metalearning/)

SET(TOOLS_SOURCES
	${SMLEARNING_ROOT}/src/tools/data_handling.cpp
	${SMLEARNING_ROOT}/src/tools/math_helpers.cpp
)

SET(SEQUENCES_TEST_SOURCE
       ${SMLEARNING_ROOT}/src/tools/sequences_test.cpp
)

SET(PARSING_TEST_SOURCE
        ${SMLEARNING_ROOT}/src/tools/parsing_test.cpp
)

SET(N_FOLD_CVSETS_NETS_SOURCE
        ${SMLEARNING_ROOT}/src/tools/n_fold_cvsets_nets.cpp
)

SET(CONCATENATE_SEQUENCEDATASETS_SOURCE
        ${SMLEARNING_ROOT}/src/tools/concatenate_sequencedatasets.cpp
)

SET(TOOLS_HEADERS
	${SMLEARNING_ROOT}/include/tools/data_handling.h
	${SMLEARNING_ROOT}/include/tools/math_helpers.h
)

SET(CONFIG_FILES
	${SMLEARNING_ROOT}/config/defaultnet.config
	${SMLEARNING_ROOT}/config/offline_experiment.xml
	${SMLEARNING_ROOT}/config/train.nc
	${SMLEARNING_ROOT}/config/valid.nc
)

INSTALL(FILES ${TOOLS_HEADERS} DESTINATION include/SMLearning/tools/)

# SMLearning library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${METALEARNING_SOURCES} ${TOOLS_SOURCES})
# ADD_LIBRARY(${PROJECT_NAME} SHARED ${TOOLS_SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

ADD_EXECUTABLE(offline_experiment ${OFFLINE_EXPERIMENT_SOURCE})
ADD_EXECUTABLE(offline_experiment_ice ${OFFLINE_EXPERIMENT_ICE_SOURCE})
ADD_EXECUTABLE(ensemble_off_experiment ${ENSEMBLE_OFF_EXPERIMENT_SOURCE})
ADD_EXECUTABLE(sequences_test ${SEQUENCES_TEST_SOURCE})
ADD_EXECUTABLE(parsing_test ${PARSING_TEST_SOURCE})
ADD_EXECUTABLE(n_fold_cvsets_nets ${N_FOLD_CVSETS_NETS_SOURCE})
ADD_EXECUTABLE(concatenate_sequencedatasets ${CONCATENATE_SEQUENCEDATASETS_SOURCE})
INSTALL(PROGRAMS ${SMLEARNING_ROOT}/src/tools/training.sh DESTINATION bin/SMLearning)
INSTALL(TARGETS offline_experiment RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS offline_experiment_ice RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS ensemble_off_experiment RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS sequences_test RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS parsing_test RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS n_fold_cvsets_nets RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS concatenate_sequencedatasets RUNTIME DESTINATION bin/SMLearning)
INSTALL(FILES ${CONFIG_FILES} DESTINATION bin/SMLearning)

## SMLearning tools library
#ADD_EXECUTABLE(tools_test ${TOOLS_SOURCES})
#INSTALL(TARGETS tools_test RUNTIME DESTINATION bin/SMLearning)
#ADD_LIBRARY(${PROJECT_NAME}_tools SHARED ${TOOLS_SOURCES})
#INSTALL(TARGETS ${PROJECT_NAME}_tools LIBRARY DESTINATION lib)

