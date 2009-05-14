#
# BALT - The Boxes and Lines Toolkit for component communication.
# 
# Copyright (C) 2006-2007 Nick Hawes, Gregor Berginc
# 
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1 of
# the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
# 02110-1301 USA
#

IF (NOT COSY_ROOT)

   #   MESSAGE("setting COSY_ROOT") 

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

# Find OmniORB package.
FIND_PACKAGE(OmniORB REQUIRED)

IF (OMNIORB4_FOUND)

  # Make sure to include all files from cosy.
  INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/c++)
  INCLUDE_DIRECTORIES(${OUTPUT}/include)
  LINK_DIRECTORIES(${OUTPUT}/lib)

  # You can prepare verbose Makefile to see what's going.
  SET(CMAKE_VERBOSE_MAKEFILE true)

  INCLUDE(UseIDL)

  # Specify the list of subdirectories to process.
  ADD_SUBDIRECTORY(src/c++/balt/idl)
  ADD_SUBDIRECTORY(src/c++/balt/core)
  ADD_SUBDIRECTORY(src/c++/balt/interface)
  ADD_SUBDIRECTORY(src/c++/balt/jni)

ENDIF (OMNIORB4_FOUND)



IF (NOT OMNIORB4_FOUND)
  MESSAGE("-------------------------------------------------------")
  MESSAGE("OmniORB installation was not found.")
  MESSAGE("Please ensure pkg-config can find omniORB4.pc,")
  MESSAGE("so that `pkg-config --libs omniORB4` returns something.") 
  MESSAGE("-------------------------------------------------------")
ENDIF (NOT OMNIORB4_FOUND)
