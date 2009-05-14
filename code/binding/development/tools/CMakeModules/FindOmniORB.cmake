#
# Find the omniORB libraries and include dir
#
 
# OMNIORB4_INCLUDE_DIR  - Directories to include to use omniORB
# OMNIORB4_LIBRARIES    - Files to link against to use omniORB
# OMNIORB4_IDL_COMPILER
# OMNIORB4_FOUND        - When false, don't try to use omniORB
# OMNIORB4_DIR          - (optional) Suggested installation directory to search
#
# OMNIORB4_DIR can be used to make it simpler to find the various include
# directories and compiled libraries when omniORB was not installed in the
# usual/well-known directories (e.g. because you made an in tree-source
# compilation or because you installed it in an "unusual" directory).
# Just set OMNIORB4_DIR it to your specific installation directory

# Include the pkg-config binary.
INCLUDE(UsePkgConfig)

# Try to find the omniidl compiler. 
FIND_PROGRAM(OMNIORB4_IDL_COMPILER
  NAMES omniidl
  PATHS ${OMNIORB4_DIR}/bin
	/usr/local/cosy
        /usr/bin
        /usr/local/bin
	/Users/luser/packages/bin
  DOC "What is the path where omniidl (the idl compiler) can be found"
)

# Find omniORB linker and compiler flags.
PKGCONFIG("omniORB4 omnithread3 omniDynamic4 omniCOSDynamic4 omniCOS4" OMNIORB4_INCLUDE_DIR OMNIORB4_LINK_DIR OMNIORB4_LINK_FLAGS OMNIORB4_CFLAGS)

IF (OMNIORB4_IDL_COMPILER)
	IF (OMNIORB4_LINK_FLAGS)
		IF (OMNIORB4_CFLAGS)
			SET( OMNIORB4_FOUND "YES" )

			MARK_AS_ADVANCED( OMNIORB4_LINK_FLAGS )
			MARK_AS_ADVANCED( OMNIORB4_CFLAGS )
			MARK_AS_ADVANCED( OMNIORB4_IDL_COMPILER )

			SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OMNIORB4_CFLAGS}")
			SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${OMNIORB4_LINK_FLAGS}")
		ENDIF (OMNIORB4_CFLAGS)
	ENDIF (OMNIORB4_LINK_FLAGS)
ENDIF (OMNIORB4_IDL_COMPILER)

