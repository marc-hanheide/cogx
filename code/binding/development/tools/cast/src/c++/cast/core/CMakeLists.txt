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


PROJECT(CASTCore)

SET(SOURCES CASTException.cpp InspectableComponent.cpp CASTUtils.cpp
	CASTComponent.cpp CASTUIComponent.cpp
	CASTProcessingComponent.cpp ArchitectureConfiguration.cpp
	CASTWorkingMemoryItem.cpp CASTWorkingMemory.cpp
	CASTComponentPermissionsMap.cpp CASTWMPermissionsMap.cpp)

SET(HEADERS CASTException.hpp InspectableComponent.hpp CASTUtils.hpp
	CASTComponent.hpp CASTUIComponent.hpp
	CASTProcessingComponent.hpp CASTCore.hpp
	CASTDatatypeManager.hpp CASTTypedData.hpp CASTData.hpp
	CASTDataCache.hpp CASTDataLocalCache.hpp
	ArchitectureConfiguration.hpp CASTWorkingMemoryInterface.hpp
	CASTWorkingMemoryItem.hpp CASTWorkingMemory.hpp DrawBuffer.hpp
	QueuedDataThread.hpp CASTComponentPermissionsMap.hpp
	CASTWMPermissionsMap.hpp)

ADD_LIBRARY(CASTCore SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(CASTCore ${OMNIORB4_LIBRARIES})
TARGET_LINK_LIBRARIES(CASTCore CASTIDL FrameworkCore FrameworkInterface)


INSTALL(TARGETS CASTCore LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/cast/core)
