# Set some common Matlab paths.
set (MATLAB_PATH
	/usr/local/cosy/matlab
	/usr/local/matlab
	/opt/matlab
	/usr/matlab
	${MATLAB}
)

# TODO Add more different architecture names.
set (ARCH_SUFFIXES
	bin/glnx86
)

macro (FIND_MATLAB_LIB _var _name)
	FIND_LIBRARY(${_var}
		NAMES ${_name} 
		PATHS ${MATLAB_PATH}
		PATH_SUFFIXES ${ARCH_SUFFIXES}	
	)
endmacro (FIND_MATLAB_LIB _var _name)

# Look for Matlab's main include file, engine.h. If the engine.h file is found,
# the MATLAB_HOME will point to the location your Matlab installation.
find_path (MATLAB_INCLUDE_DIR
	NAMES engine.h
	PATHS ${MATLAB_PATH}
	PATH_SUFFIXES extern/include
)


find_program(MATLAB_BIN_MCC mcc)
find_program(MATLAB_BIN_MBUILD mbuild)
mark_as_advanced(MATLAB_BIN_MCC)
mark_as_advanced(MATLAB_BIN_MBUILD)

if (NOT MATLAB_BIN_MCC OR NOT MATLAB_BIN_MBUILD)
   message("Matlab Compiler (mcc+mbuild) was not found.")
#~    SET(MATLAB_DIR "" CACHE PATH "Root of Matlab instal tree.")
endif (NOT MATLAB_BIN_MCC OR NOT MATLAB_BIN_MBUILD)

include_directories( ${MATLAB_INCLUDE_DIR} )

add_subdirectory(prjdeploy)
add_subdirectory(Proxy)
