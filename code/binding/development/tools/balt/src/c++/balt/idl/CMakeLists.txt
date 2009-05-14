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

PROJECT(FrameworkIDL)


# Set all IDL sources.
SET(IDLS 
	${IDL_ROOT}/FrameworkBasics.idl 
	${IDL_ROOT}/FrameworkData.idl 
	${IDL_ROOT}/RemoteConnector.idl
)

# Send IDLs to omniidl compiler. Since we want to apply different package
# renames to different IDLs we can't just use the FOREACH command. Please, note
# that the ${IDLS} variable above is necessary for a successful installation!
#
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/FrameworkBasics.idl)
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/FrameworkData.idl)
#ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/RemoteConnector.idl 
#	-pnFrameworkBasics=balt/idl/FrameworkBasics 
#	-pnFrameworkData=balt/idl/FrameworkData)

ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/RemoteConnector.idl)

ADD_IDL_MAPPING(FrameworkBasics balt/idl/FrameworkBasics)
ADD_IDL_MAPPING(FrameworkData balt/idl/FrameworkData)

	
ADD_LIBRARY(FrameworkIDL SHARED ${SOURCES})


#ADD_CUSTOM_COMMAND(TARGET FrameworkIDL
#			  PRE_BUILD	
#			  COMMAND ant
#			  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

#make it depend on java so that java gets built first
#ADD_DEPENDENCIES(FrameworkIDL balt-java)


TARGET_LINK_LIBRARIES(FrameworkIDL ${OMNIORB4_LIBRARIES})

INSTALL(TARGETS FrameworkIDL LIBRARY DESTINATION lib)
INSTALL(FILES ${AUTOGEN_HEADERS} DESTINATION include/balt/idl)
INSTALL(FILES ${IDLS} DESTINATION include/idl)
