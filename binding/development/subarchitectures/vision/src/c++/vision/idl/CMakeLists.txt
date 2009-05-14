PROJECT(VisionIDL)

# Set all IDL sources.
SET(IDLS ${IDL_ROOT}/Vision.idl)

# Send IDLs to omniidl compiler. Since we want to apply different package
# renames to different IDLs we can't just use the FOREACH command. Please, note
# that the ${IDLS} variable above is necessary for a successful installation!
INCLUDE(${CMAKE_MODULE_PATH}/UseIDL.cmake)
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/Vision.idl)
ADD_IDL_MAPPING(Vision vision/idl/Vision)
	#-pnFrameworkBasics=balt/idl/FrameworkBasics 
	#-pnFrameworkData=balt/idl/FrameworkData
	#-pnCAST=caat/idl/CAST
	#-pnMatlab=mlhelper/idl/Matlab
	#-pnMath=CoSyCommon/idl/Math)

ADD_LIBRARY(VisionIDL SHARED ${SOURCES})

#make it depend on the java code, just for a laugh... (nick)
#ADD_DEPENDENCIES(VisionIDL vision-java)


TARGET_LINK_LIBRARIES(VisionIDL ${OMNIORB4_LIBRARIES})
TARGET_LINK_LIBRARIES(VisionIDL MLHelperIDL)
TARGET_LINK_LIBRARIES(VisionIDL CoSyCommonIDLs)
TARGET_LINK_LIBRARIES(VisionIDL FrameworkIDL)

INSTALL(TARGETS VisionIDL LIBRARY DESTINATION lib)
INSTALL(FILES ${AUTOGEN_HEADERS} DESTINATION include/vision/idl)
INSTALL(FILES ${IDLS} DESTINATION include/idl)
