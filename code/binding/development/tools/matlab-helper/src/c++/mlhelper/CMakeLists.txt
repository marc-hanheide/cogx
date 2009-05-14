PROJECT(MLHelper)

SET(SOURCES uol_matlab_engine.cpp MatlabHelper.cpp)
SET(HEADERS uol_matlab_engine.h MatlabHelper.h)

#INCLUDE_DIRECTORIES(/usr/local/matlab/extern/include)
#LINK_DIRECTORIES(/usr/local/matlab/bin/glnx86)

ADD_LIBRARY(MLHelper SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(MLHelper ${OMNIORB4_LIBRARIES})

TARGET_LINK_LIBRARIES(MLHelper MLHelperIDL)

# Add Matlab related stuff.
INCLUDE_DIRECTORIES(${MATLAB_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(MLHelper ${MATLAB_LIBS})

#TARGET_LINK_LIBRARIES(MLHelper eng mx ut mat icudata icui18n icuuc ustdio)

INSTALL(TARGETS MLHelper LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/mlhelper)
INSTALL(FILES ${AUTOGEN_HEADERS} DESTINATION include/mlhelper/idl)
