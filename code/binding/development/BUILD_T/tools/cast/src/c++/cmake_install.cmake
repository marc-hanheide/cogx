# Install script for directory: /home/plison/svn.cogx/binding/development/tools/cast/src/c++

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Debug")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/cast" TYPE FILE FILES
    "/home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/cast.hpp"
    "/home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/core.hpp"
    "/home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture.hpp"
    "/home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/server.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/slice/cmake_install.cmake")
  INCLUDE("/home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/core/cmake_install.cmake")
  INCLUDE("/home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/architecture/cmake_install.cmake")
  INCLUDE("/home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/examples/cmake_install.cmake")
  INCLUDE("/home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/examples/comedyarch/cmake_install.cmake")
  INCLUDE("/home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/server/cmake_install.cmake")
  INCLUDE("/home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/testing/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

