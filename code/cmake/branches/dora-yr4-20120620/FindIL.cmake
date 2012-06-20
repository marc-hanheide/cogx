#
# Find the libdevil (IL) c++ libraries and include dir
#
 
# Include the pkg-config binary.
find_package(PkgConfig)

# # Find player linker and compiler flags.
pkg_check_modules(IL REQUIRED IL)

