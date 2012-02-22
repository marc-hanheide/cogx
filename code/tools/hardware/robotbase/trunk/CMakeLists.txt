project (ROBOTBASE)

# use cmake files packaged with CogX and CAST
include_directories(${COGX_ROOT}/tools/d-lib/src/c++/d)

# make a "use" cmake include too
include_directories(${CURE_INSTALLROOT}/include/cure)

# auto generated code
add_subdirectory (src/c++/autogen)
if (DO_SLICE_ONLY)
   return()
endif()

add_subdirectory (src/c++/components)

