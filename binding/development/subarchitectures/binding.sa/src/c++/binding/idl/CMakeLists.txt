PROJECT(BindingIDL)



# Set all IDL sources.
SET(IDLS ${IDL_ROOT}/BindingData.idl ${IDL_ROOT}/BindingFeaturesCommon.idl ${IDL_ROOT}/BindingFeatures.idl ${IDL_ROOT}/BindingQueries.idl )


# Send IDLs to omniidl compiler. Since we want to apply different package
# renames to different IDLs we can't just use the FOREACH command. Please, note
# that the ${IDLS} variable above is necessary for a successful installation!
INCLUDE(${CMAKE_MODULE_PATH}/UseIDL.cmake)
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/BindingData.idl)
ADD_IDL_MAPPING(BindingData binding/idl/BindingData)
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/BindingFeaturesCommon.idl)
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/BindingFeatures.idl)
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/BindingQueries.idl)
ADD_IDL_MAPPING(BindingFeaturesCommon binding/idl/BindingFeaturesCommon)


ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LINK_FLAGS} ${OMNIORB4_CFLAGS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} FrameworkIDL CASTIDL CoSyCommonIDLs)


INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${AUTOGEN_HEADERS} DESTINATION include/binding/idl)
INSTALL(FILES ${IDLS} DESTINATION include/idl)
