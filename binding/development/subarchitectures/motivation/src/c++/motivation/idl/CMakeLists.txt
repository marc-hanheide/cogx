PROJECT(MotivationData)

# Set all IDL sources.
SET(IDLS ${IDL_ROOT}/MotivationData.idl)

# Send IDLs to omniidl compiler. Since we want to apply different package
# renames to different IDLs we can't just use the FOREACH command. Please, note
# that the ${IDLS} variable above is necessary for a successful installation!
INCLUDE(${CMAKE_MODULE_PATH}/UseIDL.cmake)
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/MotivationData.idl)
ADD_IDL_MAPPING(MotivationData motivation/idl/MotivationData)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LINK_FLAGS} ${OMNIORB4_CFLAGS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} FrameworkIDL CASTIDL PlanningData)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PlanningData)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${AUTOGEN_HEADERS} DESTINATION include/motivation/idl)
INSTALL(FILES ${IDLS} DESTINATION include/idl)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LINK_FLAGS} ${OMNIORB4_CFLAGS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} FrameworkInterface FrameworkIDL FrameworkCore)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CASTCore CASTArchitecture CASTIDL)
	