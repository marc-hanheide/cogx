PROJECT(ComedyEssentialsIDL)

# Set all IDL sources.
SET(IDLS ${IDL_ROOT}/ComedyEssentials.idl)

# Send IDLs to omniidl compiler. Since we want to apply different package
# renames to different IDLs we can't just use the FOREACH command. Please, note
# that the ${IDLS} variable above is necessary for a successful installation!
INCLUDE(${CMAKE_MODULE_PATH}/UseIDL.cmake)
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/ComedyEssentials.idl)

ADD_LIBRARY(ComedyEssentialsIDL SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(ComedyEssentialsIDL CASTIDL)
TARGET_LINK_LIBRARIES(ComedyEssentialsIDL ${OMNIORB4_LIBRARIES})

INSTALL(TARGETS ComedyEssentialsIDL LIBRARY DESTINATION lib)
INSTALL(FILES ${AUTOGEN_HEADERS} DESTINATION include/comedyarch/idl)
INSTALL(FILES ${IDLS} DESTINATION include/idl)
