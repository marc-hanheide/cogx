PROJECT(libcrfh)

# Set build type
# SET(CMAKE_BUILD_TYPE Release)

# Add definition with version
ADD_DEFINITIONS(-DVERSION="0.6.3")

# Depend upon QT4 
FIND_PACKAGE( Qt4 REQUIRED )
INCLUDE(${QT_USE_FILE})

# Applications and libraries
SET(LIBCRFH_SOURCES  src/libCRFH.cpp src/CImage.cpp src/global.cpp src/CMatrix.cpp src/CFilter.cpp
                     src/CFilterCache.cpp src/CPerformance.cpp src/CChannelCache.cpp src/CSystem.cpp
                     src/CSparseVector.cpp src/CDescriptorList.cpp
                     src/CDescriptor.cpp src/CScaleSpaceCache.cpp src/CCrfh.cpp)
SET(LIBCRFH_HEADERS  src/libCRFH.h src/CImage.h src/global.h src/CMatrix.h src/CFilter.h src/CFilterCache.h
                     src/CPerformance.h src/CChannelCache.h src/CSystem.h src/CSparseVector.h
                     src/CDescriptorList.h src/CDescriptor.h src/CScaleSpaceCache.h
                     src/CCrfh.h)
ADD_LIBRARY(crfh SHARED ${LIBCRFH_SOURCES})
TARGET_LINK_LIBRARIES(crfh ${QT_LIBRARIES})

ADD_EXECUTABLE(crfh-extract src/crfh-extract.cpp)
TARGET_LINK_LIBRARIES(crfh-extract crfh)

# Installing
INSTALL(TARGETS crfh DESTINATION lib)
INSTALL(TARGETS crfh-extract DESTINATION bin)

