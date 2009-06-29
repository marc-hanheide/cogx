# Install script for directory: /home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture

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
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libCASTArchitecture.so")
      FILE(RPATH_CHECK
           FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libCASTArchitecture.so"
           RPATH "")
    ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libCASTArchitecture.so")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cast" TYPE SHARED_LIBRARY FILES "/home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/architecture/libCASTArchitecture.so")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libCASTArchitecture.so")
      FILE(RPATH_REMOVE
           FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libCASTArchitecture.so")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libCASTArchitecture.so")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libCASTArchitecture.so")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/cast/architecture" TYPE FILE FILES
    "/home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture/WorkingMemoryAttachedComponent.hpp"
    "/home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture/WorkingMemoryWriterComponent.hpp"
    "/home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture/WorkingMemoryReaderComponent.hpp"
    "/home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture/ManagedComponent.hpp"
    "/home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture/SubarchitectureTaskManager.hpp"
    "/home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture/ChangeFilterFactory.hpp"
    "/home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture/UnmanagedComponent.hpp"
    "/home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture/WorkingMemoryChangeFilterMap.hpp"
    "/home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture/WorkingMemoryChangeFilterComparator.hpp"
    "/home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture/WorkingMemoryChangeReceiver.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libSubarchitectureWorkingMemory.so")
      FILE(RPATH_CHECK
           FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libSubarchitectureWorkingMemory.so"
           RPATH "")
    ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libSubarchitectureWorkingMemory.so")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cast" TYPE SHARED_LIBRARY FILES "/home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/architecture/libSubarchitectureWorkingMemory.so")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libSubarchitectureWorkingMemory.so")
      FILE(RPATH_REMOVE
           FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libSubarchitectureWorkingMemory.so")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libSubarchitectureWorkingMemory.so")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libSubarchitectureWorkingMemory.so")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/cast/architecture" TYPE FILE FILES "/home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture/SubarchitectureWorkingMemory.hpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libAlwaysPositiveTaskManager.so")
      FILE(RPATH_CHECK
           FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libAlwaysPositiveTaskManager.so"
           RPATH "")
    ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libAlwaysPositiveTaskManager.so")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cast" TYPE SHARED_LIBRARY FILES "/home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/architecture/libAlwaysPositiveTaskManager.so")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libAlwaysPositiveTaskManager.so")
      FILE(RPATH_REMOVE
           FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libAlwaysPositiveTaskManager.so")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libAlwaysPositiveTaskManager.so")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cast/libAlwaysPositiveTaskManager.so")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/cast/architecture" TYPE FILE FILES "/home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture/AlwaysPositiveTaskManager.hpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")

