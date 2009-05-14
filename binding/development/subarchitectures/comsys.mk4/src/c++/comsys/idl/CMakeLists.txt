PROJECT(ComsysEssentialsIDL)



# Set all IDL sources.
SET(IDLS ${IDL_ROOT}/LFEssentials.idl ${IDL_ROOT}/LFPacking.idl ${IDL_ROOT}/ComsysEssentials.idl)


# Send IDLs to omniidl compiler. Since we want to apply different package
# renames to different IDLs we can't just use the FOREACH command. Please, note
# that the ${IDLS} variable above is necessary for a successful installation!
INCLUDE(${CMAKE_MODULE_PATH}/UseIDL.cmake)
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/LFEssentials.idl)
ADD_IDL_MAPPING(LFEssentials comsys/idl/LFEssentials)
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/LFPacking.idl)
ADD_IDL_MAPPING(LFPacking comsys/idl/LFPacking)
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/ComsysEssentials.idl)
ADD_IDL_MAPPING(ComsysEssentials comsys/idl/ComsysEssentials)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

#INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/c++)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LINK_FLAGS} ${OMNIORB4_CFLAGS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} FrameworkIDL CASTIDL CoSyCommonIDLs)


INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${AUTOGEN_HEADERS} DESTINATION include/comsys/idl)
INSTALL(FILES ${IDLS} DESTINATION include/idl)

