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

include(${CAST_ROOT}/cmake/UseBoost.cmake)


PROJECT(CASTTesting)
SET(SOURCES AbstractTester.cpp)
SET(HEADERS AbstractTester.hpp)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CASTArchitecture) # CASTIDL)
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} pthread)
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib/cast)
install(FILES ${headers} DESTINATION include/cast/testing)

add_cast_component_internal(BasicTester BasicTester.cpp BasicTester.hpp)
TARGET_LINK_LIBRARIES(BasicTester CASTTesting)

add_cast_component_internal(LockTester LockTester.cpp LockTester.hpp)
TARGET_LINK_LIBRARIES(LockTester CASTTesting)


# PROJECT(Proposer)
# SET(SOURCES Proposer.cpp)
# SET(HEADERS Proposer.hpp)
# ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
# TARGET_LINK_LIBRARIES(${PROJECT_NAME} TestTaskManager)
# INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
# INSTALL(FILES ${HEADERS} DESTINATION include/cast/testing)

# PROJECT(DiabolicTester)
# SET(SOURCES DiabolicTester.cpp)
# SET(HEADERS DiabolicTester.hpp)
# ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
# TARGET_LINK_LIBRARIES(${PROJECT_NAME} TestTaskManager)
# INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
# INSTALL(FILES ${HEADERS} DESTINATION include/cast/testing)

