
# Try to find V4R in tools.
# Look in all directories tools/*/ for the presence of the subdirectory v4r/TomGine
# which is part of the v4r library:
if ( "x${V4R_DIR}x" STREQUAL "xx"  )
  file(GLOB V4R_TOMGINE ${COGX_ROOT}/tools/*/v4r/TomGine)
  #message(" * TOM TOM: " ${V4R_TOMGINE})
  if (EXISTS ${V4R_TOMGINE})
    get_filename_component(V4R_DIR ${V4R_TOMGINE}/../.. ABSOLUTE)
  endif()
  #message(" * DIR RID: " ${V4R_DIR})
endif()

# If V4R_DIR is not set try to set it from an environment variable
if ( "x${V4R_DIR}x" STREQUAL "xx"  )
  set( V4R_DIR "$ENV{V4R_DIR}" )
endif()

message( STATUS " * V4R_DIR is '${V4R_DIR}'")
if (NOT EXISTS ${V4R_DIR}  )
  message( FATAL_ERROR
    "The library v4r (from TUW ACIN) could not be found.\n"
    "Your options:\n"
    "  a) check out the library into ${COGX_ROOT}/tools\n"
    "  b) check out the library into another directory and set the V4R_DIR environment variable\n"
    "     eg. in your .profile file with export V4R_DIR=\"$HOME/projects/acin\"n"
    )
endif()

set (V4R_LIBRARY_DIRS ${V4R_DIR}/lib)
set (V4R_INCLUDE_DIRS ${V4R_DIR})
link_directories(${V4R_LIBRARY_DIRS})
include_directories(${V4R_INCLUDE_DIRS})
