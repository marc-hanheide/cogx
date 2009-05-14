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

PROJECT(FrameworkInterface)

# Set the sources.  

SET(SOURCES LocalConnectionManager.cpp RemoteDatatypeManager.cpp
RemoteTranslatorManager.cpp BALTTimer.cpp)

SET(HEADERS BALTInterface.hpp includes.hpp LocalConnectionManager.hpp
	RemotePushConnectors.hpp RemotePullConnectors.hpp
	RemoteDataTranslator.hpp RemoteDatatypeManager.hpp
	GenericRemoteConnectionCreator.hpp RemoteConnectionCreator.hpp
	RemoteTranslatorManager.hpp BALTTimer.hpp)

# Add the FrameworkInterface library to the build targets.
ADD_LIBRARY(FrameworkInterface SHARED ${SOURCES})

#TARGET_LINK_LIBRARIES(FrameworkInterface ${OMNIORB4_LIBRARIES})

# Add libraries to link against.
TARGET_LINK_LIBRARIES(FrameworkInterface FrameworkCore FrameworkIDL)

INSTALL(TARGETS FrameworkInterface LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/balt/interface)
