PROJECT(DLIB)

# add_subdirectory(src/c++/autogen)
if (DO_SLICE_ONLY)
   return()
endif()

# we need these
include(UseCAST)


ADD_SUBDIRECTORY(src/c++/d)
