PROJECT(MLHelperIDL)

# Set all IDL sources.
SET(IDLS ${IDL_ROOT}/Matlab.idl)

# Send IDLs to omniidl compiler. Since we want to apply different package
# renames to different IDLs we can't just use the FOREACH command. Please, note
# that the ${IDLS} variable above is necessary for a successful installation!
INCLUDE(${CMAKE_MODULE_PATH}/UseIDL.cmake)

ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/Matlab.idl)
ADD_IDL_MAPPING(Matlab mlhelper/idl/Matlab)

ADD_LIBRARY(MLHelperIDL SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(MLHelperIDL ${OMNIORB4_LIBRARIES})

INSTALL(TARGETS MLHelperIDL LIBRARY DESTINATION lib)
INSTALL(FILES ${AUTOGEN_HEADERS} DESTINATION include/mlhelper/idl)
INSTALL(FILES ${IDLS} DESTINATION include/idl)
