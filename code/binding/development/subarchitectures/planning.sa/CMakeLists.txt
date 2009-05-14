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
SET(IDL_ROOT ${CMAKE_SOURCE_DIR}/src/idl)

# Make sure to include all files from cosy.
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/c++)
INCLUDE_DIRECTORIES(${OUTPUT}/include)
LINK_DIRECTORIES(${OUTPUT}/lib)

# You can prepare verbose Makefile to see what's going.
#SET(CMAKE_VERBOSE_MAKEFILE true)
INCLUDE_DIRECTORIES(src/c++/planning)

#nah: always build idl and ontology as they are needed widely
ADD_SUBDIRECTORY(src/c++/planning/idl)
ADD_SUBDIRECTORY(src/c++/planning/util)
ADD_SUBDIRECTORY(src/c++/planning/components/abstr)
ADD_SUBDIRECTORY(src/c++/planning/components/Trigger)
ADD_SUBDIRECTORY(src/c++/planning/components/SimpleStateGenerator)
ADD_SUBDIRECTORY(src/c++/planning/components/SimpleActionGenerator)


#ADD_SUBDIRECTORY(src/c++/planning/components/PlanningBindingWorkingMemory)
