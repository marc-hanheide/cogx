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

PROJECT(FrameworkJNI)


# Set the sources.
SET(SOURCES JNIProcessInterface.cpp)

# Set the JNI classes to build stubs from.
SET(JNI_CLASSES balt.jni.NativeProcessLauncher)

# Find the javah executable.
FIND_PACKAGE(Javah)
# If javah is found, we can build the header and the stub from java class file.
IF (JAVAH_FOUND)
	# If the jni.h was found, include its location - this is something for the
	# MACs.
	# Compile all jni classes to get the stubs.
	FOREACH (jni_class ${JNI_CLASSES})
		# Change dots with underscores to prepare the c++ class name.
		STRING(REGEX REPLACE "\\." "_" cpp_class ${jni_class})

		ADD_CUSTOM_COMMAND(
			OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${cpp_class}.h
			COMMAND ${JAVAH_RUNTIME}
			ARGS -verbose -classpath ${OUTPUT}/classes -d ${CMAKE_CURRENT_SOURCE_DIR} ${jni_class}
		)

		SET(AUTOGEN_HEADERS ${AUTOGEN_HEADERS} ${CMAKE_CURRENT_SOURCE_DIR}/${cpp_class}.h)
	ENDFOREACH (jni_class ${JNI_CLASSES})
ENDIF (JAVAH_FOUND)

INCLUDE_DIRECTORIES(${JAVAH_INCLUDE_DIR} ${JAVAH_INCLUDE_DIR}/linux)

# Add the FrameworkJNI library.
ADD_LIBRARY(FrameworkJNI SHARED ${SOURCES} ${AUTOGEN_HEADERS})


IF (${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
	SET_TARGET_PROPERTIES(FrameworkJNI PROPERTIES SUFFIX .jnilib)	
ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")

TARGET_LINK_LIBRARIES(FrameworkJNI ${OMNIORB4_LIBRARIES})

# Add libraries to link against.
TARGET_LINK_LIBRARIES(FrameworkJNI FrameworkCore FrameworkInterface FrameworkIDL)

INSTALL(TARGETS FrameworkJNI LIBRARY DESTINATION lib)
INSTALL(FILES ${AUTOGEN_HEADERS} DESTINATION include/balt/jni)
