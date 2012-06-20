project(CASTUtils)

include(UseCAST)

# add_subdirectory(src/c++/autogen)
if (DO_SLICE_ONLY)
   return()
endif()

add_subdirectory(src/c++)
