cmake_minimum_required (VERSION 2.6)
project (Alchemy)

# add_subdirectory(src/c++/autogen)
if (DO_SLICE_ONLY)
   return()
endif()

if(APPLE)
	set(ALCHEMY_MAKEFILE makefile_MAC)
else()
	set(ALCHEMY_MAKEFILE makefile)
endif()

add_custom_target(alchemy ALL
	COMMAND make -f ${ALCHEMY_MAKEFILE}
	WORKING_DIRECTORY ${COGX_ROOT}/tools/alchemy/src
	COMMENT "Building alchemy"
)
