# Install script for directory: /home/plison/svn.cogx/binding/development/tools/cast/cmake

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cast/cmake" TYPE FILE FILES
    "/home/plison/svn.cogx/binding/development/tools/cast/cmake/CASTBuild.cmake"
    "/home/plison/svn.cogx/binding/development/tools/cast/cmake/CASTDeps.cmake"
    "/home/plison/svn.cogx/binding/development/tools/cast/cmake/FindIce.cmake"
    "/home/plison/svn.cogx/binding/development/tools/cast/cmake/Slice2Cpp.cmake"
    "/home/plison/svn.cogx/binding/development/tools/cast/cmake/UseBoost.cmake"
    "/home/plison/svn.cogx/binding/development/tools/cast/cmake/UseCAST.cmake"
    "/home/plison/svn.cogx/binding/development/tools/cast/cmake/UseIce.cmake"
    "/home/plison/svn.cogx/binding/development/tools/cast/cmake/FindCAST.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cast/templates" TYPE FILE FILES
    "/home/plison/svn.cogx/binding/development/tools/cast/cmake/ComponentTemplateCMakeLists.txt"
    "/home/plison/svn.cogx/binding/development/tools/cast/cmake/ProjectTemplateCMakeLists.txt"
    "/home/plison/svn.cogx/binding/development/tools/cast/cmake/TopLevelTemplateCMakeLists.txt"
    "/home/plison/svn.cogx/binding/development/tools/cast/cmake/build-template.xml"
    "/home/plison/svn.cogx/binding/development/tools/cast/cmake/build-toplevel.xml"
    "/home/plison/svn.cogx/binding/development/tools/cast/cmake/slice2java-template.xml"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")

