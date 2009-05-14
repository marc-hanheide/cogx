#
# CAST - The CoSy Architecture Schema Toolkit
#
# Copyright (C) 2006-2007 Michael Zillich, Nick Hawes, Gregor Berginc
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

PROJECT(GUIDemo)

SET(SOURCES GUIDemo.cpp)
SET(HEADERS GUIDemo.hpp)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

#SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-Wl,-E")

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} FrameworkInterface FrameworkIDL FrameworkCore)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CASTCore CASTArchitecture CASTIDL)


INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/vision/components/GUIDemo)
