PROJECT(CoSyCommonIDLs)

# Set all IDL sources.
SET(IDLS ${IDL_ROOT}/Math.idl)

# Send IDLs to omniidl compiler. Since we want to apply different package
# renames to different IDLs we can't just use the FOREACH command. Please, note
# that the ${IDLS} variable above is necessary for a successful installation!
INCLUDE(${CMAKE_MODULE_PATH}/UseIDL.cmake)

ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/Math.idl)
ADD_IDL_MAPPING(Math CoSyCommon/idl/Math)
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/VisualAttributes.idl)

ADD_LIBRARY(CoSyCommonIDLs SHARED ${SOURCES})

#ADD_DEPENDENCIES(CoSyCommonIDLs idl-java)

TARGET_LINK_LIBRARIES(CoSyCommonIDLs ${OMNIORB4_LIBRARIES})

INSTALL(TARGETS CoSyCommonIDLs LIBRARY DESTINATION lib)
INSTALL(FILES ${AUTOGEN_HEADERS} DESTINATION include/CoSyCommon/idl)
INSTALL(FILES ${IDLS} DESTINATION include/idl)
