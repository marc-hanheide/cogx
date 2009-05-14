PROJECT(TestingIDL)

# Set all IDL sources
SET(IDLS ${IDL_ROOT}/TestingData.idl)

# Send IDLs to omniidl compiler. Since we want to apply different package
# renames to different IDLs we can't just use the FOREACH command. Please, note
# that the ${IDLS} variable above is necessary for a successful installation!
INCLUDE(${CMAKE_MODULE_PATH}/UseIDL.cmake)
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/TestingData.idl)
ADD_IDL_MAPPING(Testing testing/idl/TestingData)

ADD_LIBRARY(TestingIDL SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(TestingIDL ${OMNIORB4_LIBRARIES})
TARGET_LINK_LIBRARIES(TestingIDL FrameworkIDL)

INSTALL(TARGETS TestingIDL LIBRARY DESTINATION lib)
INSTALL(FILES ${AUTOGEN_HEADERS} DESTINATION include/testing/idl)
INSTALL(FILES ${IDLS} DESTINATION include/idl)

