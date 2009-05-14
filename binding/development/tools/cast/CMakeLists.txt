IF (NOT COSY_ROOT)

   # Set the CoSy root.
   GET_FILENAME_COMPONENT(COSY_ROOT ../.. ABSOLUTE)

   # Set the output directory.
   SET(OUTPUT ${COSY_ROOT}/output)

ENDIF (NOT COSY_ROOT)


# Set the output directory.
SET(CMAKE_INSTALL_PREFIX ${OUTPUT} CACHE INTERNAL "")

# Set the path to additional CMake modules (e.g., our FindOmniORB module).
# SET(CMAKE_MODULE_PATH ${COSY_ROOT}/tools/CMakeModules)

# Set the path where other thing should be relative to
GET_FILENAME_COMPONENT(CMAKE_SOURCE_DIR . ABSOLUTE CACHE INTERNAL "")


# Set c++ and idl root.
SET(CPP_ROOT ${CMAKE_SOURCE_DIR}/src/c++)
SET(IDL_ROOT ${CMAKE_SOURCE_DIR}/src/idl)

# Make sure to include all files from cosy.
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/c++)
INCLUDE_DIRECTORIES(${OUTPUT}/include)
LINK_DIRECTORIES(${OUTPUT}/lib)

# You can prepare verbose Makefile to see what's going.
SET(CMAKE_VERBOSE_MAKEFILE true)

# Find OmniORB package.
FIND_PACKAGE(OmniORB REQUIRED)
#get boost too
FIND_PACKAGE(Boost REQUIRED)
 
IF (OMNIORB4_FOUND)
IF (Boost_FOUND)

  INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
  #LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

  INCLUDE(UseIDL)

  # Specify the list of subdirectories to process.
  ADD_SUBDIRECTORY(src/c++/cast/cdl)
  ADD_SUBDIRECTORY(src/c++/cast/ui)
  ADD_SUBDIRECTORY(src/c++/cast/core)
  ADD_SUBDIRECTORY(src/c++/cast/architecture)
  ADD_SUBDIRECTORY(src/c++/cast/testing)

ENDIF (Boost_FOUND)
ENDIF (OMNIORB4_FOUND)

IF (NOT Boost_FOUND)
  MESSAGE("-------------------------------------------------------")
  MESSAGE("Boost installation was not found, please install it and set")
  MESSAGE("an environment variable called BOOST_ROOT to point to the root")
  MESSAGE("of where you have installed Boost")
  MESSAGE("-------------------------------------------------------")
ENDIF (NOT Boost_FOUND)
