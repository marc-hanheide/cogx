OPTION(BINDING_BUILD_COMSYS_TESTER "Build the binding tester for comsys" NO)
OPTION(BINDING_BUILD_COMA_TESTER "Build the binding tester for coma" NO)
OPTION(BINDING_BUILD_EXPERIMENTAL_FEATURES "Includes some extra binding features for experimantal purposes" NO)
OPTION(BINDING_DISABLE_TOKENS "Disable using tokens for synchronization" NO)

IF (BINDING_BUILD_EXPERIMENTAL_FEATURES)
  ADD_DEFINITIONS(-DEXPERIMENTAL_FEATURES)
ENDIF (BINDING_BUILD_EXPERIMENTAL_FEATURES)

IF (NOT COSY_ROOT)
   # Set the CoSy root... MAKE SURE THIS POINTS TO THE code DIR
   GET_FILENAME_COMPONENT(COSY_ROOT ../.. ABSOLUTE)

   # Set the output directory.
   SET(OUTPUT ${COSY_ROOT}/output)

ENDIF (NOT COSY_ROOT)

# Set the install directory.
SET(CMAKE_INSTALL_PREFIX ${OUTPUT} CACHE INTERNAL "")

# Set the path to additional CMake modules (e.g., our FindOmniORB module).
SET(CMAKE_MODULE_PATH ${COSY_ROOT}/tools/CMakeModules)

# Set the path where other thing should be relative to
GET_FILENAME_COMPONENT(CMAKE_SOURCE_DIR . ABSOLUTE CACHE INTERNAL "")

# Find OmniORB package.
FIND_PACKAGE(OmniORB)

# Set c++ and idl root.
SET(CPP_ROOT ${CMAKE_SOURCE_DIR}/src/c++)
SET(SLICE_ROOT ${CMAKE_SOURCE_DIR}/src/slice)

# Make sure to include all files from cosy.
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/c++)
INCLUDE_DIRECTORIES(${OUTPUT}/include)
LINK_DIRECTORIES(${OUTPUT}/lib)

# You can prepare verbose Makefile to see what's going.
#SET(CMAKE_VERBOSE_MAKEFILE true)

FIND_PACKAGE(Boost)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

INCLUDE_DIRECTORIES(src/c++/)

IF (BINDING_BUILD_COMSYS_TESTER)
  ADD_DEFINITIONS(-DCOMSYS_INCLUDED)
  ADD_SUBDIRECTORY(src/c++/binding/components/ComsysTester)
ENDIF (BINDING_BUILD_COMSYS_TESTER)

IF (BINDING_BUILD_COMA_TESTER)
  ADD_DEFINITIONS(-DCOMA_INCLUDED)
  ADD_SUBDIRECTORY(src/c++/binding/components/ComaTester)
ENDIF (BINDING_BUILD_COMA_TESTER)

IF (BINDING_DISABLE_TOKENS)
  ADD_DEFINITIONS(-DNBINDING_TOKENS)
ENDIF (BINDING_DISABLE_TOKENS)


ADD_SUBDIRECTORY(src/c++/binding/slice)
ADD_SUBDIRECTORY(src/c++/binding)
ADD_SUBDIRECTORY(src/c++/binding/components/MultiTesterMonitor)
ADD_SUBDIRECTORY(src/c++/binding/components/GarbageCollector)
ADD_SUBDIRECTORY(src/c++/binding/components/TesterMonitor)
ADD_SUBDIRECTORY(src/c++/binding/components/AmbiguityIdentifier)
ADD_SUBDIRECTORY(src/c++/binding/components/BindingScorer)
ADD_SUBDIRECTORY(src/c++/binding/components/BindingDotViewer)
ADD_SUBDIRECTORY(src/c++/binding/components/BinderComponent)
ADD_SUBDIRECTORY(src/c++/binding/components/BindingJudge)
ADD_SUBDIRECTORY(src/c++/binding/components/BindingWorkingMemory)
ADD_SUBDIRECTORY(src/c++/binding/components/BindingStatusMonitor)
ADD_SUBDIRECTORY(src/c++/binding/components/BindingGroupManager)
ADD_SUBDIRECTORY(src/c++/binding/components/FeatureComparatorExample)
#deprecated ADD_SUBDIRECTORY(src/c++/binding/components/CopyCatBindingMonitor)
#probably wrong now ADD_SUBDIRECTORY(src/c++/binding/components/FeatureGenerator)

