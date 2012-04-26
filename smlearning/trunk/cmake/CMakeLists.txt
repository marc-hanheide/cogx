cmake_minimum_required(VERSION 2.6)
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# TrackerTest exec ---------------------------
#PROJECT(TrackerTest)
#SET(SOURCES /home/cogx/svn.cogx/code/smlearning/trunk/src/Scenario/TrackerThread.cpp)
#LINK_LIBRARIES(cv highgui GL GLU GLEW ftgl)
#LINK_LIBRARIES(GLWindow Tracker TomGine ThreadObject)
#ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCES})
#INSTALL(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)



# SMLearning
PROJECT(SMLearning)


# Project root directory
GET_FILENAME_COMPONENT(PROJECT_ROOT ../.. ABSOLUTE)
SET(SMLEARNING_ROOT ${PROJECT_ROOT}/trunk CACHE INTERNAL "Path prefix for SMLearning package")

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

#SET(KNI_INCLUDE /usr/local/include/KNI CACHE PATH "Path prefix for KNI include")
#SET(KNI_LIBRARY /usr/local/lib CACHE PATH "Path prefix for KNI library")

##### NEW ##### ##### NEW ##### ##### NEW #####
# Katana drivers for Golem are with the Golem executables
SET(GOLEM_INCLUDE /usr/local/include/Golem CACHE PATH "Path prefix for Golem include")
SET(GOLEM_LIBRARY /usr/local/lib CACHE PATH "Path prefix for Golem library")

SET(RNNLIB_INCLUDE /usr/local/include/rnnlib CACHE PATH "Path prefix for RNNLIB include")
SET(RNNLIB_LIBRARY /usr/local/lib CACHE PATH "Path prefix for RNNLIB library")

SET(BLORT_INCLUDE /usr/local/include/Blort CACHE PATH "Path prefix for BLORT include files")
SET(BLORT_LIBRARY /usr/local/lib CACHE PATH "Path prefix for BLORT libraries")

SET(CRYSSMEX_INCLUDE /usr/local/include/cryssmex CACHE PATH "Path prefix for CrySSMEx include")
SET(CRYSSMEX_LIBRARY /usr/local/lib CACHE PATH "Path prefix for CrySSMEx library")

SET(NEURALGAS_INCLUDE /usr/local/include/NeuralGas CACHE PATH "Path prefix for NEURALGAS include")
SET(NEURALGAS_LIBRARY /usr/local/lib CACHE PATH "Path prefix for NEURALGAS library")

SET(Boost_MAJOR_VERSION 1)
SET(Boost_MINOR_VERSION 42)
# SET(Boost_SUBMINOR_VERSION 0)
FIND_PACKAGE( Boost 1.42 COMPONENTS filesystem regex program_options)

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

FIND_PACKAGE( Qt4 REQUIRED )

SET(QWT_INCLUDE /usr/include/qwt-qt4 CACHE PATH "Path prefix for qwt include")
SET(QWT_LIBRARY /usr/lib/ CACHE PATH "Path prefix for qwt library")

INCLUDE( ${QT_USE_FILE} )

SET(ICE_INCLUDE /usr/include CACHE PATH "Path prefix for ZeroC Ice include")
SET(ICE_LIBRARY /usr/lib CACHE PATH "Path prefix for ZeroC Ice library")
SET(ICE_BIN /usr/bin CACHE PATH "Path prefix for ZeroC Ice binaries")

SET(SMLEARNING_INCLUDE ${SMLEARNING_ROOT}/include CACHE PATH "Path prefix for SMLearning include")

ADD_DEFINITIONS(-DLINUX -fpermissive -DNX_DISABLE_FLUIDS)
# ADD_DEFINITIONS(-DLINUX -DNX_DISABLE_FLUIDS)
LINK_LIBRARIES(NxCharacter NxCooking PhysXCore PhysXLoader #KNI
               GolemSys GolemCtrl GolemPhys GolemPhysCtrl GolemTools #GolemTinyIce !!not created anymore
	       GolemDemoCommon
	       pthread rt GL glut GLU expat SMLearning netcdf rnnlib cryssmexlib 
 	       ${Boost_FILESYSTEM_LIBRARY} ${Boost_REGEX_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY}
	       ${QT_LIBRARIES} qwt-qt4
	       cv highgui GLEW ftgl
	       TomGine Tracker GLWindow ThreadObject
	       Ice IceUtil)

INCLUDE_DIRECTORIES(
	${SMLEARNING_INCLUDE}
	${SMLEARNING_ROOT}/slice
	${GOLEM_INCLUDE}
	${GOLEM_INCLUDE}/demo/common
	${PHYSX_INCLUDE}/SDKs/Physics/include
	${PHYSX_INCLUDE}/SDKs/Foundation/include
	${PHYSX_INCLUDE}/SDKs/PhysXLoader/include
	${PHYSX_INCLUDE}/SDKs/Cooking/include
	#${KNI_INCLUDE}
	${Boost_INCLUDE_DIRS}
	${RNNLIB_INCLUDE}
	${NEURALGAS_INCLUDE}
	${CRYSSMEX_INCLUDE}
	${QT_INCLUDE_DIR}
	${QWT_INCLUDE}
	${BLORT_INCLUDE}
	${ICE_INCLUDE}
)

LINK_DIRECTORIES(
	${PHYSX_LIBRARY}
	#${KNI_LIBRARY}
	${GOLEM_LIBRARY}
	${Boost_LIBRARY_DIRS}
	${RNNLIB_LIBRARY}
	${NEURALGAS_LIBRARY}
	${CRYSSMEX_LIBRARY}
	${QWT_LIBRARY}
	${ICE_LIBRARY}
	${BLORT_LIBRARY}
)

SET(METALEARNING_SOURCES
	${SMLEARNING_ROOT}/src/metalearning/RNN.cpp
	${SMLEARNING_ROOT}/src/metalearning/ActiveRNN.cpp
	${SMLEARNING_ROOT}/src/metalearning/SMRegion.cpp
	${SMLEARNING_ROOT}/src/metalearning/GNGSMRegion.cpp
#	${SMLEARNING_ROOT}/src/metalearning/Scenario.cpp
#	${SMLEARNING_ROOT}/src/metalearning/ScenarioIce.cpp
#	${SMLEARNING_ROOT}/src/metalearning/ActiveLearnScenarioIce.cpp
#	${SMLEARNING_ROOT}/src/metalearning/ActiveLearnScenario.cpp
	${SMLEARNING_ROOT}/src/metalearning/data_structs.cpp
	${SMLEARNING_ROOT}/src/metalearning/CrySSMEx.cpp
)

# SET(OFFLINE_EXPERIMENT_SOURCE
# 	${SMLEARNING_ROOT}/src/metalearning/offline_experiment.cpp
# )

#SET(OFFLINE_EXPERIMENT_ICE_SOURCE
#	${SMLEARNING_ROOT}/src/metalearning/offline_experiment_ice.cpp
#)

#SET(ACTIVELEARN_EXPERIMENT_ICE_SOURCE
#	${SMLEARNING_ROOT}/src/metalearning/activelearn_experiment_ice.cpp
#)

# SET(ACTIVELEARN_EXPERIMENT_SOURCE
# 	${SMLEARNING_ROOT}/src/metalearning/activelearn_experiment.cpp
# )


SET(METALEARNING_HEADERS
	${SMLEARNING_ROOT}/include/metalearning/RNN.h
	${SMLEARNING_ROOT}/include/metalearning/ActiveRNN.h
	${SMLEARNING_ROOT}/include/metalearning/SMRegion.h
	${SMLEARNING_ROOT}/include/metalearning/GNGSMRegion.h
#	${SMLEARNING_ROOT}/include/metalearning/Scenario.h
#	${SMLEARNING_ROOT}/include/metalearning/ScenarioIce.h
#	${SMLEARNING_ROOT}/include/metalearning/ActiveLearnScenarioIce.h
#	${SMLEARNING_ROOT}/include/metalearning/ActiveLearnScenario.h
	${SMLEARNING_ROOT}/include/metalearning/data_structs.h
	${SMLEARNING_ROOT}/include/metalearning/CrySSMEx.h
)
INSTALL(FILES ${METALEARNING_HEADERS} DESTINATION include/SMLearning/metalearning/)

SET(SCENARIO_SOURCES
	${SMLEARNING_ROOT}/src/scenario/ActorObject.cpp
	${SMLEARNING_ROOT}/src/scenario/ArmActor.cpp
	${SMLEARNING_ROOT}/src/metalearning/data_structs.cpp
	${SMLEARNING_ROOT}/src/scenario/Scenario.cpp	
	${SMLEARNING_ROOT}/src/scenario/PredictingScenario.cpp
#	${SMLEARNING_ROOT}/src/scenario/ActiveGNGScenario.cpp
	${SMLEARNING_ROOT}/src/scenario/ActiveLearnScenario.cpp
	${SMLEARNING_ROOT}/src/scenario/PushingApplication.cpp
	${SMLEARNING_ROOT}/src/scenario/TrackerThread.cpp
)


SET(SCENARIO_HEADERS
	${SMLEARNING_ROOT}/include/scenario/ActorObject.h
	${SMLEARNING_ROOT}/include/scenario/ArmActor.h
	${SMLEARNING_ROOT}/include/scenario/ConcreteActor.h
	${SMLEARNING_ROOT}/include/metalearning/data_structs.h
	${SMLEARNING_ROOT}/include/scenario/Polyflap.h
	${SMLEARNING_ROOT}/include/scenario/PushingApplication.h
	${SMLEARNING_ROOT}/include/scenario/Scenario.h
	${SMLEARNING_ROOT}/include/scenario/PredictingScenario.h
#	${SMLEARNING_ROOT}/include/scenario/ActiveGNGScenario.h
	${SMLEARNING_ROOT}/include/scenario/ActiveLearnScenario.h
	${SMLEARNING_ROOT}/include/scenario/TrackerThread.h
)

INSTALL(FILES ${SCENARIO_HEADERS} DESTINATION include/SMLearning/scenario/)

SET(PREDICTION_EXPERIMENT_SOURCE
	${SMLEARNING_ROOT}/src/scenario/prediction_experiment.cpp
)

SET(OFFLINE_EXPERIMENT_SOURCE
	${SMLEARNING_ROOT}/src/scenario/offline_experiment.cpp
)

# SET(ACTIVEGNG_EXPERIMENT_SOURCE
# 	${SMLEARNING_ROOT}/src/scenario/activegng_experiment.cpp
# )
SET(ACTIVELEARN_EXPERIMENT_SOURCE
	${SMLEARNING_ROOT}/src/scenario/activelearn_experiment.cpp
)

SET(OFFLINEACTIVEGNG_EXPERIMENT_SOURCE
	${SMLEARNING_ROOT}/src/metalearning/offlineactivegng_experiment.cpp
)

SET(TRACKER_TEST_SOURCE
 	${SMLEARNING_ROOT}/src/tests/TrackerTest.cpp
)

SET(TOOLS_SOURCES
	${SMLEARNING_ROOT}/src/smltools/helpers.cpp
	${SMLEARNING_ROOT}/src/smltools/math_helpers.cpp
	${SMLEARNING_ROOT}/src/smltools/data_plot.cpp
)

SET(SEQUENCES_TEST_SOURCE
       ${SMLEARNING_ROOT}/src/tests/sequences_test.cpp
)

SET(PARSING_TEST_SOURCE
        ${SMLEARNING_ROOT}/src/tests/parsing_test.cpp
)

SET(DATA_PLOT_TEST_SOURCE
        ${SMLEARNING_ROOT}/src/tests/data_plot_test.cpp
)

# SET(CANONICAL_INPUT_OUTPUT_ENUMERATOR_TEST_SOURCE
#         ${SMLEARNING_ROOT}/src/tools/canonical_input_output_enumerator_test.cpp
# )

SET(PRINT_DATASET_SOURCE
        ${SMLEARNING_ROOT}/src/smltools/print_dataset.cpp
)

SET(EVAL_PLOTTING_SOURCE
        ${SMLEARNING_ROOT}/src/smltools/eval_plotting.cpp
)

SET(EVALUATE_ACTIVELEARN_SOURCE
        ${SMLEARNING_ROOT}/src/metalearning/evaluate_activelearn.cpp
)

SET(EVALUATE_OFFLINELEARN_SOURCE
        ${SMLEARNING_ROOT}/src/metalearning/evaluate_offlinelearn.cpp
)


SET(N_FOLD_CVSETS_NETS_SOURCE
        ${SMLEARNING_ROOT}/src/smltools/n_fold_cvsets_nets.cpp
)

SET(CONCATENATE_SEQUENCEDATASETS_SOURCE
        ${SMLEARNING_ROOT}/src/smltools/concatenate_sequencedatasets.cpp
)

SET(WRITE_DATA_CRYSSMEX_FMT_SOURCE
        ${SMLEARNING_ROOT}/src/smltools/write_data_cryssmex_fmt.cpp
)

SET(TOOLS_HEADERS
	${SMLEARNING_ROOT}/include/smltools/helpers.h
	${SMLEARNING_ROOT}/include/smltools/math_helpers.h
	${SMLEARNING_ROOT}/include/smltools/tracker_tools.h
)
INSTALL(FILES ${TOOLS_HEADERS} DESTINATION include/SMLearning/tools/)

SET(DATA_PLOT_MOC_HEADERS
	${SMLEARNING_ROOT}/include/smltools/data_plot.h
)
QT4_WRAP_CPP( DATA_PLOT_GEN_MOC ${DATA_PLOT_MOC_HEADERS} )

SET(DATA_PLOT_TEST_MOC_HEADERS
	${SMLEARNING_ROOT}/include/tests/data_plot_test.h
)
QT4_WRAP_CPP( DATA_PLOT_TEST_GEN_MOC ${DATA_PLOT_TEST_MOC_HEADERS} )

SET(CONFIG_FILES
	${SMLEARNING_ROOT}/config/defaultnet.config
	${SMLEARNING_ROOT}/config/offline_experiment.xml
	${SMLEARNING_ROOT}/config/prediction_experiment.xml
#	${SMLEARNING_ROOT}/config/activegng_experiment.xml
	${SMLEARNING_ROOT}/config/activelearn_experiment.xml
	${SMLEARNING_ROOT}/config/train.nc
	${SMLEARNING_ROOT}/config/valid.nc
#	${SMLEARNING_ROOT}/config/GolemDeviceKatana300Sim.xml  #For now this is not really useful
#	${SMLEARNING_ROOT}/config/GolemDeviceKatana300.xml
#	${SMLEARNING_ROOT}/config/katana300_6M180.cfg
)


###
###PlotApp Ice interface
###
SET(PLOTAPP_MOC_HEADERS
	${SMLEARNING_ROOT}/include/smltools/PlotAppI.h
)
QT4_WRAP_CPP( PLOTAPP_GEN_MOC ${PLOTAPP_MOC_HEADERS} )

SET(PLOTAPP_HEADERS
        ${SMLEARNING_ROOT}/slice/PlotApp.hh
)
SET(PLOTAPP_SOURCES
        ${SMLEARNING_ROOT}/slice/PlotApp.cc
)
ADD_CUSTOM_COMMAND(
        OUTPUT ${SMLEARNING_ROOT}/slice/PlotApp.cc ${SMLEARNING_ROOT}/slice/PlotApp.hh
	COMMAND ${ICE_BIN}/slice2cpp
	ARGS --header-ext hh --source-ext cc --output-dir ${SMLEARNING_ROOT}/slice ${SMLEARNING_ROOT}/slice/PlotApp.ice
	DEPENDS ${SMLEARNING_ROOT}/slice/PlotApp.ice
	COMMENT "slice2cpp"
)

SET(DATA_PLOT_TEST_SERVER_SOURCE
        ${SMLEARNING_ROOT}/src/tests/data_plot_test_server.cpp
        ${SMLEARNING_ROOT}/slice/PlotApp.cc
)

SET(DATA_PLOT_TEST_CLIENT_SOURCE
        ${SMLEARNING_ROOT}/src/tests/data_plot_test_client.cpp
        ${SMLEARNING_ROOT}/slice/PlotApp.cc
)

SET(PLOT_SOURCES
        ${PLOTAPP_SOURCES}
	${DATA_PLOT_GEN_MOC}
	${PLOTAPP_GEN_MOC}
)

SET(DATA_PLOT_SERVER_SOURCE
        ${SMLEARNING_ROOT}/src/smltools/data_plot_server.cpp
        ${SMLEARNING_ROOT}/slice/PlotApp.cc
)


# SMLearning library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${METALEARNING_SOURCES} ${TOOLS_SOURCES} ${PLOT_SOURCES} ${SCENARIO_SOURCES} ${TRACKER_SOURCES})
# ADD_LIBRARY(${PROJECT_NAME} SHARED ${TOOLS_SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

ADD_EXECUTABLE(offline_experiment ${OFFLINE_EXPERIMENT_SOURCE})
#ADD_EXECUTABLE(offline_experiment_ice ${OFFLINE_EXPERIMENT_ICE_SOURCE})
#ADD_EXECUTABLE(activelearn_experiment_ice ${ACTIVELEARN_EXPERIMENT_ICE_SOURCE})
ADD_EXECUTABLE(activelearn_experiment ${ACTIVELEARN_EXPERIMENT_SOURCE})
ADD_EXECUTABLE(sequences_test ${SEQUENCES_TEST_SOURCE})
ADD_EXECUTABLE(parsing_test ${PARSING_TEST_SOURCE})
ADD_EXECUTABLE(data_plot_test ${DATA_PLOT_TEST_SOURCE} ${DATA_PLOT_TEST_GEN_MOC})
# ADD_EXECUTABLE(canonical_input_output_enumerator_test ${CANONICAL_INPUT_OUTPUT_ENUMERATOR_TEST_SOURCE})
ADD_EXECUTABLE(print_dataset ${PRINT_DATASET_SOURCE})
ADD_EXECUTABLE(eval_plotting ${EVAL_PLOTTING_SOURCE})
ADD_EXECUTABLE(evaluate_activelearn ${EVALUATE_ACTIVELEARN_SOURCE})
ADD_EXECUTABLE(evaluate_offlinelearn ${EVALUATE_OFFLINELEARN_SOURCE})
ADD_EXECUTABLE(n_fold_cvsets_nets ${N_FOLD_CVSETS_NETS_SOURCE})
ADD_EXECUTABLE(concatenate_sequencedatasets ${CONCATENATE_SEQUENCEDATASETS_SOURCE})
ADD_EXECUTABLE(write_data_cryssmex_fmt ${WRITE_DATA_CRYSSMEX_FMT_SOURCE})
ADD_EXECUTABLE(data_plot_test_server ${DATA_PLOT_TEST_SERVER_SOURCE} ${DATA_PLOT_TEST_GEN_MOC})
ADD_EXECUTABLE(data_plot_test_client ${DATA_PLOT_TEST_CLIENT_SOURCE})
ADD_EXECUTABLE(data_plot_server ${DATA_PLOT_SERVER_SOURCE})
ADD_EXECUTABLE(tracker_test ${TRACKER_TEST_SOURCE})
ADD_EXECUTABLE(prediction_experiment ${PREDICTION_EXPERIMENT_SOURCE})
# ADD_EXECUTABLE(activegng_experiment ${ACTIVEGNG_EXPERIMENT_SOURCE})
ADD_EXECUTABLE(offlineactivegng_experiment ${OFFLINEACTIVEGNG_EXPERIMENT_SOURCE})

INSTALL(PROGRAMS ${SMLEARNING_ROOT}/src/scripts/training.sh DESTINATION bin/SMLearning)
INSTALL(PROGRAMS ${SMLEARNING_ROOT}/src/scripts/ensemble_experiment.sh DESTINATION bin/SMLearning)
INSTALL(TARGETS offline_experiment RUNTIME DESTINATION bin/SMLearning)
#INSTALL(TARGETS offline_experiment_ice RUNTIME DESTINATION bin/SMLearning)
#INSTALL(TARGETS activelearn_experiment_ice RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS activelearn_experiment RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS prediction_experiment RUNTIME DESTINATION bin/SMLearning)
# INSTALL(TARGETS activegng_experiment RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS offlineactivegng_experiment RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS tracker_test RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS print_dataset RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS eval_plotting RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS evaluate_activelearn RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS evaluate_offlinelearn RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS n_fold_cvsets_nets RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS concatenate_sequencedatasets RUNTIME DESTINATION bin/SMLearning)
INSTALL(TARGETS write_data_cryssmex_fmt RUNTIME DESTINATION bin/SMLearning)
INSTALL(FILES ${CONFIG_FILES} DESTINATION bin/SMLearning)
INSTALL(TARGETS data_plot_server RUNTIME DESTINATION bin/SMLearning)
##### NEW ##### ##### NEW ##### ##### NEW #####
INSTALL(FILES ${GOLEM_KATANA_DRIVERS_FILES} DESTINATION bin/SMLearning)


## SMLearning tools library
#ADD_EXECUTABLE(tools_test ${TOOLS_SOURCES})
#INSTALL(TARGETS tools_test RUNTIME DESTINATION bin/SMLearning)
#ADD_LIBRARY(${PROJECT_NAME}_tools SHARED ${TOOLS_SOURCES})
#INSTALL(TARGETS ${PROJECT_NAME}_tools LIBRARY DESTINATION lib)
