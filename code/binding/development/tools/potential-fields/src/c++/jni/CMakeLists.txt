PROJECT(PotentialFieldsJNI)


# Set the sources.
SET(SOURCES ProximityMapJNI.cpp ProjectiveMapJNI.cpp)

# Set the JNI classes to build stubs from.  

SET(JNI_CLASSES org.cognitivesystems.spatial.pf.ProximityMap
org.cognitivesystems.spatial.pf.ProjectiveMap)

# Find the javah executable.
FIND_PACKAGE(Javah)
# If javah is found, we can build the header and the stub from java class file.
IF (JAVAH_FOUND)
	# If the jni.h was found, include its location - this is something for the
	# MACs.
	# Compile all jni classes to get the stubs.
	FOREACH (jni_class ${JNI_CLASSES})
		# Change dots with underscores to prepare the c++ class name.
		STRING(REGEX REPLACE "\\." "_" cpp_class ${jni_class})

		ADD_CUSTOM_COMMAND(
			OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${cpp_class}.h
			COMMAND ${JAVAH_RUNTIME}
			ARGS -verbose -classpath ${OUTPUT}/classes -d ${CMAKE_CURRENT_SOURCE_DIR} ${jni_class}
		)

		SET(AUTOGEN_HEADERS ${AUTOGEN_HEADERS} ${CMAKE_CURRENT_SOURCE_DIR}/${cpp_class}.h)
	ENDFOREACH (jni_class ${JNI_CLASSES})
ENDIF (JAVAH_FOUND)

INCLUDE_DIRECTORIES(${JAVAH_INCLUDE_DIR} ${JAVAH_INCLUDE_DIR}/linux)

INCLUDE_DIRECTORIES(..)


# Add the library.
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES} ${AUTOGEN_HEADERS})


IF (${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
	SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES SUFFIX .jnilib)	
ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")

TARGET_LINK_LIBRARIES(${PROJECT_NAME} PotentialFields)


INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${AUTOGEN_HEADERS} DESTINATION include/pf/jni)
