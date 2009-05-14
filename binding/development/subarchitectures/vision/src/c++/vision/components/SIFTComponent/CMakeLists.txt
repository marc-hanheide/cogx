PROJECT(SIFTComponent)

SET(SOURCES SIFTComponent.cpp SiftDetectEngine.cpp
SiftCore/sift.cpp
SiftCore/ioutil.cpp
SiftCore/siftGlobal.cpp
SiftCore/Representation.cpp
SiftCore/DatabaseEntry.cpp
SiftCore/extractorI.cpp
SiftCore/matcher.cpp)

SET(HEADERS SIFTComponent.h SiftDetectEngine.h
SiftCore/sift.h
SiftCore/ioutil.h
SiftCore/siftGlobal.h
SiftCore/Recognizer.h
SiftCore/Representation.h
SiftCore/DatabaseEntry.h
SiftCore/extractorI.h
SiftCore/matcher.h)


# set path to SVS (Small Vision System)
# removed by nick... cut'n'paste?
# SET(SVS_ROOT /usr/local/packages/svs)

# HACK: imlib-config does not work with cmake
# neither:
# EXECUTE_PROCESS(COMMAND imlib-config --cflags
#                 OUTPUT_VARIABLE IMLIB_INC)
# nor:
# SET(IMLIB_CXX_FLAGS "`imlib-config --cflags`")
# Therefore do it manually (and introduce many happy hours of debugging for
# every new machine this component is installed on)
SET(IMLIB_INC /usr/X11R6/include)
SET(IMLIB_LIB /usr/X11R6/lib)

INCLUDE_DIRECTORIES(SiftCore)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LIBRARIES})

#TARGET_LINK_LIBRARIES(${PROJECT_NAME} PlanningSubarchAbstr)
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} PlanningSubarchUtil)
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} PlanningSubarch)
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} PlanningData)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} Vision VisionIDL)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} VisionUtils)

# Add VxL libraries and includes if the library itself is found.
IF (VXL_FOUND)
	# IF the VXL is found, include INCLUDE dir and libraries.
	IF (VXL_VCL_FOUND)
		INCLUDE_DIRECTORIES(${VXL_VCL_INCLUDE_DIR})
		TARGET_LINK_LIBRARIES(${PROJECT_NAME} vil vcl vimt vil_algo vnl_algo)
		TARGET_LINK_LIBRARIES(${PROJECT_NAME} vil vcl vimt vil_algo vnl_algo)
		TARGET_LINK_LIBRARIES(${PROJECT_NAME} netlib vil_io vil vsl vnl vgl vcl)
	ELSE (VXL_VCL_FOUND)
		MESSAGE("Oops, the VCL missing. Please check the VXL_DIR")
	ENDIF (VXL_VCL_FOUND)

	# If Core of the VXL is found, include it.
	IF (VXL_CORE_FOUND)
		INCLUDE_DIRECTORIES(${VXL_CORE_INCLUDE_DIR})
	ELSE (VXL_CORE_FOUND)
		MESSAGE("Oops, the VxL Core missing. Please check the VXL_DIR")
	ENDIF (VXL_CORE_FOUND)
	
	INCLUDE_DIRECTORIES(${VXL_MUL_INCLUDE_DIR})
	INCLUDE_DIRECTORIES(${VXL_GEL_INCLUDE_DIR})
  
	#MESSAGE("MUL includes " ${VXL_MUL_INCLUDE_DIR})


ENDIF (VXL_FOUND)


# Use pkg-config to find opencv libraries...
INCLUDE(UsePkgConfig)
PKGCONFIG("opencv" OCV_INCLUDE_DIR OCV_LINK_DIR OCV_LINK_FLAGS OCV_CFLAGS)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OCV_LINK_FLAGS})


INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/vision/components/SIFTComponent)
