#
# Find the player c++ libraries and include dir
#
 
# Include the pkg-config binary.
find_package(PkgConfig)

# # Find player linker and compiler flags.
pkg_check_modules(PLAYER REQUIRED playerc++>=2.1.1)


link_directories(${PLAYER_LIBRARY_DIRS})
include_directories(${PLAYER_INCLUDE_DIRS})