# -*- mode: cmake -*-
cmake_minimum_required(VERSION 2.6)

PROJECT(CDSR)

# version information
SET(RSB_VERSION_MAJOR "0")
SET(RSB_VERSION_MINOR "6")

FIND_PACKAGE(RSB "${RSB_VERSION_MAJOR}.${RSB_VERSION_MINOR}" REQUIRED)
FIND_PACKAGE(ProtocolBuffers REQUIRED)

MESSAGE(STATUS "RSB version: ${RSB_VERSION}")
MESSAGE(STATUS "RSB libraries: ${RSB_LIBRARIES}")
                      
ADD_SUBDIRECTORY(src/cpp)