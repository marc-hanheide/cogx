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

# Set c++ and idl root.
SET(CPP_ROOT ${CMAKE_SOURCE_DIR}/src/c++)
SET(IDL_ROOT ${CMAKE_SOURCE_DIR}/src/idl)

# Make sure to include all files from cosy.
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/c++)
INCLUDE_DIRECTORIES(${OUTPUT}/include)
LINK_DIRECTORIES(${OUTPUT}/lib)


INCLUDE_DIRECTORIES(${COSY_ROOT}/tools/katanad)
INCLUDE_DIRECTORIES(${COSY_ROOT}/tools/katanaclnt)

# You can prepare verbose Makefile to see what's going.
SET(CMAKE_VERBOSE_MAKEFILE true)

# Find OmniORB package.
FIND_PACKAGE(OmniORB)

# Specify the list of subdirectories to process.
ADD_SUBDIRECTORY(src/c++/manipulation/idl)
ADD_SUBDIRECTORY(src/c++/manipulation/components/PretendManipulator)

OPTION(BUILD_REAL_MANIPULATOR "Build real manipulation components" NO)


IF(BUILD_REAL_MANIPULATOR)
    ADD_SUBDIRECTORY(src/c++/manipulation/components/WorkingMemory)
    # MHN: TAKING OUT BINDING MONITOR IN MANIP FOR NOW...
    #ADD_SUBDIRECTORY(src/c++/manipulation/components/ManipulationBindingMonitor)
    ADD_SUBDIRECTORY(src/c++/manipulation/components/Manipulator)
    ADD_SUBDIRECTORY(src/c++/manipulation/components/DumbArse)
    ADD_SUBDIRECTORY(src/c++/manipulation)
ENDIF(BUILD_REAL_MANIPULATOR)

