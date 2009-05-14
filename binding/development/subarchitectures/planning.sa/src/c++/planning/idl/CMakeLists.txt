PROJECT(PlanningData)

# Set all IDL sources.
SET(IDLS ${IDL_ROOT}/PlanningData.idl ${IDL_ROOT}/Planner.idl)

# Send IDLs to omniidl compiler. Since we want to apply different package
# renames to different IDLs we can't just use the FOREACH command. Please, note
# that the ${IDLS} variable above is necessary for a successful installation!
INCLUDE(${CMAKE_MODULE_PATH}/UseIDL.cmake)

ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/Planner.idl)

ADD_IDL_MAPPING(Planner planning/idl/Planner)

ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/PlanningData.idl)

ADD_IDL_MAPPING(PlanningData planning/idl/PlanningData)


ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LINK_FLAGS} ${OMNIORB4_CFLAGS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} FrameworkIDL CASTIDL)


INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${AUTOGEN_HEADERS} DESTINATION include/planning/idl)
INSTALL(FILES ${IDLS} DESTINATION include/idl)
