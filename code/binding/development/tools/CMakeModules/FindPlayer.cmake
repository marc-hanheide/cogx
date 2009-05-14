#
# Find the player c++ libraries and include dir
#
 
# PLAYER_INCLUDE_DIR  - Directories to include to use player
# PLAYER_LIBRARIES    - Files to link against to use player
# PLAYER_FOUND        - When false, don't try to use player
# PLAYER_DIR          - (optional) Suggested installation directory to search
#
# PLAYER_DIR can be used to make it simpler to find the various include
# directories and compiled libraries when player was not installed in the
# usual/well-known directories (e.g. because you made an in tree-source
# compilation or because you installed it in an "unusual" directory).
# Just set PLAYER_DIR it to your specific installation directory

# Include the pkg-config binary.
INCLUDE(UsePkgConfig)

# # Find player linker and compiler flags.
PKGCONFIG("playerc++" PLAYER_INCLUDE_DIR PLAYER_LINK_DIR PLAYER_LINK_FLAGS PLAYER_CFLAGS)

IF (PLAYER_LINK_FLAGS)
	IF (PLAYER_CFLAGS)
		SET( PLAYER_FOUND "YES" )
		MARK_AS_ADVANCED( PLAYER_LINK_FLAGS )
		MARK_AS_ADVANCED( PLAYER_CFLAGS )

		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PLAYER_CFLAGS}")
		SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${PLAYER_LINK_FLAGS}")
	ENDIF (PLAYER_CFLAGS)
ENDIF (PLAYER_LINK_FLAGS)
