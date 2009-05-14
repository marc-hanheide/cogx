# Try to locate the c++filt runtime.
FIND_PROGRAM(C++FILT_RUNTIME
	NAMES c++filt
	PATHS $ENV{PATH}
)

IF (C++FILT_RUNTIME)
	SET (C++FILT_FOUND "YES")
	# Mark the variable as advanced so that users don't need to know about it.
	MARK_AS_ADVANCED(C++FILT_RUNTIME)
	SET (C++FILT_CMD "${C++FILT_RUNTIME} -t " CACHE STRING "correct call to c++filt for a type")
	#STRING(REGEX REPLACE "\\/" "_" C++FILT_DEF ${C++FILT_RUNTIME})
	ADD_DEFINITIONS(-D__CFILT__="\\\"${C++FILT_CMD} \\\"")
ENDIF (C++FILT_RUNTIME)

IF (NOT C++FILT_FOUND)
  MESSAGE("c++filt was not found in the path. Please provide C++FILT_RUNTIME:")
  SET(C++FILT_RUNTIME "" CACHE PATH "The path to c++filt executable." )
ENDIF (NOT C++FILT_FOUND)

