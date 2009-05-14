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
 
PROJECT(CASTArchitecture)

SET(SOURCES SubarchitectureProcessException.cpp
    ConsistencyException.cpp WMException.cpp
    DoesNotExistOnWMException.cpp PermissionException.cpp
    AlreadyExistsOnWMException.cpp
    LocalWorkingMemoryAttachedComponent.cpp
    WorkingMemoryAttachedComponent.cpp WorkingMemoryReaderProcess.cpp
    WorkingMemoryWriterProcess.cpp
    SubarchitectureWorkingMemoryProtocol.cpp ManagedProcess.cpp
    SubarchitectureTaskManager.cpp SubarchitectureWorkingMemory.cpp)

SET(HEADERS SubarchitectureProcessException.hpp
    ConsistencyException.hpp WMException.hpp
    DoesNotExistOnWMException.hpp PermissionException.hpp
    AlreadyExistsOnWMException.hpp WorkingMemoryChangeReceiver.hpp
    WorkingMemoryChangeFilterComparator.hpp
    WorkingMemoryChangeFilterMap.hpp
    LocalWorkingMemoryAttachedComponent.hpp
    WorkingMemoryAttachedComponent.hpp WorkingMemoryReaderProcess.hpp
    WorkingMemoryWriterProcess.hpp UnmanagedProcess.hpp
    ManagedProcess.hpp PrivilegedManagedProcess.hpp
    CASTArchitecture.hpp SubarchitectureWorkingMemoryProtocol.hpp
    WorkingMemoryPullQuery.hpp SubarchitectureTaskManager.hpp
    SubarchitectureWorkingMemory.hpp ChangeFilterFactory.hpp)

ADD_LIBRARY(CASTArchitecture SHARED ${SOURCES})
TARGET_LINK_LIBRARIES(CASTArchitecture CASTCore)

INSTALL(TARGETS CASTArchitecture LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/cast/architecture)

PROJECT(AlwaysPositiveTaskManager)
SET(SOURCES AlwaysPositiveTaskManager.cpp)
SET(HEADERS AlwaysPositiveTaskManager.hpp)
ADD_LIBRARY(AlwaysPositiveTaskManager SHARED ${SOURCES})
TARGET_LINK_LIBRARIES(AlwaysPositiveTaskManager CASTArchitecture)
INSTALL(TARGETS AlwaysPositiveTaskManager LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/cast/architecture)
