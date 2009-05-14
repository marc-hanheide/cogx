# Set some common Matlab paths.
SET (MATLAB_PATH
	/usr/local/cosy/matlab
	/usr/local/matlab
	/opt/matlab
	/usr/matlab
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

# Try to find all the libraries before exporting the matlab variables.
FIND_MATLAB_LIB(MATLAB_LIB_ENG eng)
#MESSAGE(${MATLAB_LIB_ENG})
FIND_MATLAB_LIB(MATLAB_LIB_MX mx)
#MESSAGE(${MATLAB_LIB_MX})
FIND_MATLAB_LIB(MATLAB_LIB_UT ut)
#MESSAGE(${MATLAB_LIB_UT})
FIND_MATLAB_LIB(MATLAB_LIB_MAT mat)
FIND_MATLAB_LIB(MATLAB_LIB_ICUDATA icudata)
FIND_MATLAB_LIB(MATLAB_LIB_ICUI18N icui18n)
FIND_MATLAB_LIB(MATLAB_LIB_ICUUC icuuc)
FIND_MATLAB_LIB(MATLAB_LIB_USTDIO ustdio)

# Now, make sure, users don't see these libraries in their guis.
MARK_AS_ADVANCED(MATLAB_LIB_ENG)
MARK_AS_ADVANCED(MATLAB_LIB_MX)
MARK_AS_ADVANCED(MATLAB_LIB_UT)
MARK_AS_ADVANCED(MATLAB_LIB_MAT)
MARK_AS_ADVANCED(MATLAB_LIB_ICUDATA)
MARK_AS_ADVANCED(MATLAB_LIB_ICUI18N)
MARK_AS_ADVANCED(MATLAB_LIB_ICUUC)
MARK_AS_ADVANCED(MATLAB_LIB_USTDIO)

# Check whether all libraries are found.
if (MATLAB_LIB_ENG)
if (MATLAB_LIB_MX)
if (MATLAB_LIB_UT)
if (MATLAB_LIB_MAT)
if (MATLAB_LIB_ICUDATA)
if (MATLAB_LIB_ICUI18N)
if (MATLAB_LIB_ICUUC)
if (MATLAB_LIB_USTDIO)
	SET (MATLAB_FOUND "YES")
	SET (MATLAB_LIBS
		${MATLAB_LIB_ENG} 
		${MATLAB_LIB_MX} 
		${MATLAB_LIB_UT} 
		${MATLAB_LIB_MAT} 
		${MATLAB_LIB_ICUDATA}
		${MATLAB_LIB_ICUI18N} 
		${MATLAB_LIB_ICUUC} 
		${MATLAB_LIB_USTDIO}
	)
	MARK_AS_ADVANCED(MATLAB_INCLUDE_DIR)
	MARK_AS_ADVANCED(MATLAB_LIBS)
endif (MATLAB_LIB_USTDIO)
endif (MATLAB_LIB_ICUUC)
endif (MATLAB_LIB_ICUI18N)
endif (MATLAB_LIB_ICUDATA)
endif (MATLAB_LIB_MAT)
endif (MATLAB_LIB_UT)
endif (MATLAB_LIB_MX)
endif (MATLAB_LIB_ENG)

IF (NOT MATLAB_FOUND)
	MESSAGE("Complete Matlab installation was not found. Please provide MATLAB_DIR:")
	MESSAGE("  - through the GUI when working with ccmake, ")
	MESSAGE("  - as a command line argument when working with cmake e.g. ")
	MESSAGE("    cmake .. -DMATLAB_DIR:PATH=/usr/local/matlab")
	SET(MATLAB_DIR "" CACHE PATH "Root of Matlab instal tree.")
ENDIF(NOT MATLAB_FOUND)

# added by nick to make a weird error go away when using this with
# other dirs... and now it appears to break it!
#SET(CMAKE_FIND_LIBRARY_PREFIXES "lib")
#SET(CMAKE_FIND_LIBRARY_SUFFIXES "so" "a" "dylib")
