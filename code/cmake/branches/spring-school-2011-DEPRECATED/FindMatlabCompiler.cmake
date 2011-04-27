find_program(MATLAB_BIN_MCC mcc)
find_program(MATLAB_BIN_MBUILD mbuild)
mark_as_advanced(MATLAB_BIN_MCC)
mark_as_advanced(MATLAB_BIN_MBUILD)

if (NOT MATLAB_BIN_MCC OR NOT MATLAB_BIN_MBUILD)
   message("Matlab Compiler (mcc+mbuild) was not found.")
   set(MATLAB_DIR "")
else (NOT MATLAB_BIN_MCC OR NOT MATLAB_BIN_MBUILD)
   # Guess matlab install root

   #~ABSOLUTE doesn't resolve links
   #~get_filename_component(MCC_ABS ${MATLAB_BIN_MCC} ABSOLUTE)

   # Try to resolve mcc symlink
   EXECUTE_PROCESS(
	  COMMAND /bin/sh -c "ls -l ${MATLAB_BIN_MCC}" 
	  COMMAND grep -oe "->\\s*.*" 
	  COMMAND sed -e "s/->\\s*//"
	  OUTPUT_VARIABLE  MCC_SYM_RESOLVED
	  ERROR_VARIABLE   COMMAND_ERROR
	  OUTPUT_STRIP_TRAILING_WHITESPACE
	  )
   if (MCC_SYM_RESOLVED)
	  get_filename_component(MATLAB_DIR ${MCC_SYM_RESOLVED} PATH)
   else(MCC_SYM_RESOLVED)
	  get_filename_component(MATLAB_DIR ${MATLAB_BIN_MCC} PATH)
   endif(MCC_SYM_RESOLVED)

   # .../matlab/bin --> .../matlab
   get_filename_component(MATLAB_DIR ${MATLAB_DIR} PATH)
endif (NOT MATLAB_BIN_MCC OR NOT MATLAB_BIN_MBUILD)
set (MATLAB_DIR ${MATLAB_DIR} CACHE PATH  "Root of Matlab instal tree.")

# Set some common Matlab paths.
set (MATLAB_PATH
	${MATLAB_DIR}
	/usr/local/matlab
	/opt/matlab
	/usr/matlab
	~/apps/matlab
	~/bin/matlab
)

# Look for Matlab's main include file, mclmcr.h. 
find_path (MATLAB_INCLUDE_DIR
	NAMES mclmcr.h
	PATHS ${MATLAB_PATH}
	PATH_SUFFIXES extern/include
)

# TODO Add more different architecture names.
#set (ARCH_SUFFIXES
#    bin/glnx86
#)

#macro (FIND_MATLAB_LIB _var _name)
#    FIND_LIBRARY(${_var}
#        NAMES ${_name} 
#        PATHS ${MATLAB_PATH}
#        PATH_SUFFIXES ${ARCH_SUFFIXES}	
#    )
#endmacro (FIND_MATLAB_LIB _var _name)

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


# added by nick to make a weird error go away when using this with
# other dirs... and now it appears to break it!
#SET(CMAKE_FIND_LIBRARY_PREFIXES "lib")
#SET(CMAKE_FIND_LIBRARY_SUFFIXES "so" "a" "dylib")
