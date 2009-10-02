# Set some common Matlab paths.
SET (MATLAB_PATH
	/usr/local/matlab
	/opt/matlab
	/usr/matlab
	/home/mmarko/apps/matlab2008a
	${MATLAB}
)

# TODO Add more different architecture names.
SET (ARCH_SUFFIXES
	bin/glnx86
)

MACRO (FIND_MATLAB_LIB _var _name)
	FIND_LIBRARY(${_var}
		NAMES ${_name} 
		PATHS ${MATLAB_PATH}
		PATH_SUFFIXES ${ARCH_SUFFIXES}	
	)
ENDMACRO (FIND_MATLAB_LIB _var _name)

# Look for Matlab's main include file, engine.h. If the engine.h file is found,
# the MATLAB_HOME will point to the location your Matlab installation.
FIND_PATH (MATLAB_INCLUDE_DIR
	NAMES engine.h
	PATHS ${MATLAB_PATH}
	PATH_SUFFIXES extern/include
)

# Try to find some libraries before exporting the matlab variables.
#~ FIND_MATLAB_LIB(MATLAB_LIB_ENG eng)
#~ FIND_MATLAB_LIB(MATLAB_LIB_MX mx)
#~ FIND_MATLAB_LIB(MATLAB_LIB_MAT mat)
#~ FIND_MATLAB_LIB(MATLAB_LIB_UT ut)

# Now, make sure, users don't see these libraries in their guis.
#~ MARK_AS_ADVANCED(MATLAB_LIB_ENG)
#~ MARK_AS_ADVANCED(MATLAB_LIB_MX)
#~ MARK_AS_ADVANCED(MATLAB_LIB_MAT)
#~ MARK_AS_ADVANCED(MATLAB_LIB_UT)

FIND_PROGRAM(MATLAB_BIN_MCC mcc)
FIND_PROGRAM(MATLAB_BIN_MBUILD mbuild)
MARK_AS_ADVANCED(MATLAB_BIN_MCC)
MARK_AS_ADVANCED(MATLAB_BIN_MBUILD)

IF (NOT MATLAB_BIN_MCC OR NOT MATLAB_BIN_MBUILD)
   MESSAGE("Matlab Compiler (mcc+mbuild) was not found.")
#~    SET(MATLAB_DIR "" CACHE PATH "Root of Matlab instal tree.")
ENDIF (NOT MATLAB_BIN_MCC OR NOT MATLAB_BIN_MBUILD)

# added by nick to make a weird error go away when using this with
# other dirs... and now it appears to break it!
#SET(CMAKE_FIND_LIBRARY_PREFIXES "lib")
#SET(CMAKE_FIND_LIBRARY_SUFFIXES "so" "a" "dylib")
