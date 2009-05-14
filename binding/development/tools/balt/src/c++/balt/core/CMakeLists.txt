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

PROJECT(FrameworkCore)

#find c++filt for demangliing c++ symbols
FIND_PACKAGE(C++Filt)

SET(SOURCES LocalDatatypeManager.cpp
FrameworkProcessCreatorManager.cpp BALTException.cpp BALTType.cpp)

SET(HEADERS BALTCore.hpp FrameworkLocalData.hpp includes.hpp
	Interfaces.hpp FrameworkProcess.hpp FrameworkQuery.hpp
	LocalConnectionCreator.hpp LocalDatatypeManager.hpp
	FrameworkProcessCreatorManager.hpp FrameworkProcessCreator.hpp
	FrameworkConnector.hpp BALTException.hpp StringMap.hpp
	BALTType.hpp TypeMap.hpp BALTLocalUtils.hpp)

ADD_LIBRARY(FrameworkCore SHARED ${SOURCES})

#for dlopen etc
TARGET_LINK_LIBRARIES(FrameworkCore dl)
TARGET_LINK_LIBRARIES(FrameworkCore ${OMNIORB4_LIBRARIES})

INSTALL(TARGETS FrameworkCore LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/balt/core)
