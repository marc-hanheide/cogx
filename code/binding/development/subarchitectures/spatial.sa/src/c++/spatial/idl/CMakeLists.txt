PROJECT(SpatialIDL)

# Set all IDL sources.
SET(IDLS ${IDL_ROOT}/SpatialData.idl)

# Send IDLs to omniidl compiler. Since we want to apply different package
# renames to different IDLs we can't just use the FOREACH command. Please, note
# that the ${IDLS} variable above is necessary for a successful installation!
ADD_IDL(SOURCES AUTOGEN_HEADERS ${IDL_ROOT}/SpatialData.idl)
ADD_IDL_MAPPING(Spatial spatial/idl/SpatialData)

ADD_LIBRARY(SpatialIDL SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(SpatialIDL ${OMNIORB4_LIBRARIES})
TARGET_LINK_LIBRARIES(SpatialIDL FrameworkIDL CoSyCommonIDLs CASTIDL)

INSTALL(TARGETS SpatialIDL LIBRARY DESTINATION lib)
INSTALL(FILES ${AUTOGEN_HEADERS} DESTINATION include/spatial/idl)
INSTALL(FILES ${IDLS} DESTINATION include/idl)
