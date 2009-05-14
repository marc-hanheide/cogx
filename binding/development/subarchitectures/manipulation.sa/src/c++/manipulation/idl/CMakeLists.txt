PROJECT(ManipulationIDL)

# Set all IDL sources.
SET(IDLS ${IDL_ROOT}/Manipulation.idl)

# Send IDLs to omniidl compiler. Since we want to apply different package
# renames to different IDLs we can't just use the FOREACH command. Please, note
# that the ${IDLS} variable above is necessary for a successful installation!
INCLUDE(${CMAKE_MODULE_PATH}/UseIDL.cmake)
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/Manipulation.idl)

ADD_LIBRARY(ManipulationIDL SHARED ${SOURCES})

ADD_DEPENDENCIES(ManipulationIDL CoSyCommonIDLs)

TARGET_LINK_LIBRARIES(ManipulationIDL ${OMNIORB4_LIBRARIES})
TARGET_LINK_LIBRARIES(ManipulationIDL MLHelperIDL CoSyCommonIDLs CASTIDL)

INSTALL(TARGETS ManipulationIDL LIBRARY DESTINATION lib)
INSTALL(FILES ${AUTOGEN_HEADERS} DESTINATION include/manipulation/idl)
INSTALL(FILES ${IDLS} DESTINATION include/idl)
