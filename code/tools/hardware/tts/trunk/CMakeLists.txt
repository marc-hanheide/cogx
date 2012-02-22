cmake_minimum_required (VERSION 2.6)
project (TTS)


# auto generated code
add_subdirectory (src/c++/autogen)
if (DO_SLICE_ONLY)
   return()
endif()

add_subdirectory (src/c++/components)

