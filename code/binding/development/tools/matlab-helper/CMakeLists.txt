# added by nick to make a weird error go away when using this with
# other dirs... and now it appears to break it!
SET(CMAKE_FIND_LIBRARY_PREFIXES "lib")
SET(CMAKE_FIND_LIBRARY_SUFFIXES "so" "a" "dylib") 

IF (NOT COSY_ROOT)

   # Set the CoSy root.
   GET_FILENAME_COMPONENT(COSY_ROOT ../.. ABSOLUTE)

   # Set the output directory.
   SET(OUTPUT ${COSY_ROOT}/output)

ENDIF (NOT COSY_ROOT)

# Set the output directory.
SET(CMAKE_INSTALL_PREFIX ${OUTPUT} CACHE INTERNAL "")

# Set the path to additional CMake modules (e.g., our FindOmniORB module).
SET(CMAKE_MODULE_PATH ${COSY_ROOT}/tools/CMakeModules)

# Set the source dir for this build
SET(CMAKE_SOURCE_DIR ${COSY_ROOT}/tools/matlab-helper CACHE INTERNAL "")


# Set c++ and idl root.
SET(CPP_ROOT ${CMAKE_SOURCE_DIR}/src/c++)
SET(IDL_ROOT ${CMAKE_SOURCE_DIR}/src/idl)

# Make sure to include all files from cosy.
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/c++)
INCLUDE_DIRECTORIES(${OUTPUT}/include)
LINK_DIRECTORIES(${OUTPUT}/lib)

# You can prepare verbose Makefile to see what's going.
#SET(CMAKE_VERBOSE_MAKEFILE true)

# Find omniorb.
FIND_PACKAGE(OmniORB)

FIND_PACKAGE(Matlab)

# Specify the list of subdirectories to process.
ADD_SUBDIRECTORY(src/c++/mlhelper/idl)

# Build the mlhelper only if Matlab is found!
IF (MATLAB_FOUND)
	ADD_SUBDIRECTORY(src/c++/mlhelper)
ENDIF (MATLAB_FOUND)
