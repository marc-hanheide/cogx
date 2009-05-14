#
# CAST - The CoSy Architecture Schema Toolkit
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
#


PROJECT(CASTIDL)

# Set all IDL sources.
SET(IDLS ${IDL_ROOT}/CAST.idl)

# Send IDLs to omniidl compiler. Since we want to apply different package
# renames to different IDLs we can't just use the FOREACH command. Please, note
# that the ${IDLS} variable above is necessary for a successful installation!
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/CAST.idl)
# maintain old header file for the time being
ADD_IDL_MAPPING(cast cast/cdl/CAST)
ADD_IDL_MAPPING(CAST cast/cdl/CAST)

ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/guitypes.idl)
ADD_IDL_MAPPING(guitypes cast/cdl/guitypes)

ADD_LIBRARY(CASTIDL SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(CASTIDL ${OMNIORB4_LIBRARIES})
TARGET_LINK_LIBRARIES(CASTIDL FrameworkIDL)

INSTALL(TARGETS CASTIDL LIBRARY DESTINATION lib)
INSTALL(FILES ${AUTOGEN_HEADERS} DESTINATION include/cast/cdl)
INSTALL(FILES ${IDLS} DESTINATION include/idl)
