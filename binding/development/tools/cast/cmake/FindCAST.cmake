#
# Find CAST and all that stuff
#

# Variables read by this module:
#
# CAST_ROOT         - (optional) Suggested installation directory to search
#
#
# Variables set by this module 
# CAST_FOUND        - System has CAST
# CAST_INCLUDE_DIR  - The CAST include director
# CAST_LIBRARIES    - Link to these to use CAST
# CAST_CMAKE_DIR    - The directory containing the CAST cmake files
# CAST_LIBRARY_DIR  - The directory containing the CAST shared libraries
# CAST_SLICE_DIR    - The directory containing the CAST slice files

# Try to find the omniidl compiler. 
FIND_PATH(CAST_INSTALL_ROOT
  NAMES include/cast/core.hpp
  PATHS $ENV{CAST_ROOT}
	/usr/local
	/opt
	/opt/local
  DOC "The install prefix used for CAST. E.g. /usr/local. Set CAST_ROOT to reflect this on your machine if CAST is not found."
)

IF (CAST_INSTALL_ROOT)
  SET (CAST_FOUND "YES")
  SET (CAST_CMAKE_DIR "${CAST_INSTALL_ROOT}/share/cast/cmake")
  SET (CAST_INCLUDE_DIR "${CAST_INSTALL_ROOT}/include")
  SET (CAST_LIBRARY_DIR "${CAST_INSTALL_ROOT}/lib/cast")
  SET (CAST_LIBRARIES CDL CASTCore CASTArchitecture)
  SET (CAST_SLICE_DIR "${CAST_INSTALL_ROOT}/share")
ENDIF (CAST_INSTALL_ROOT)

IF (NOT CAST_INSTALL_ROOT)
  MESSAGE("A CAST installation was not found.")
  MESSAGE("You can set the CAST_ROOT env variable to point to the prefix of your installation, e.g. /usr/local, so that $CAST_ROOT/include/cast/core.hpp exists.")
  MESSAGE("You can also set")
  MESSAGE("  - through the GUI when working with ccmake, ")
  MESSAGE("  - as a command line argument when working with cmake e.g. ")
  MESSAGE("    cmake .. -DCAST_INSTALL_ROOT=/opt/cast ")
  SET(CAST_INSTALLROOT "" CACHE PATH "The path to cosycast installation." )
ENDIF (NOT CAST_INSTALL_ROOT)

