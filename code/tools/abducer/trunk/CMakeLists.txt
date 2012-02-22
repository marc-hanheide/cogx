cmake_minimum_required (VERSION 2.6)

project (Abducer)

# add_subdirectory (src/c++/autogen)
if (DO_SLICE_ONLY)
   return()
endif()

add_custom_target (Abducer ALL)
add_custom_command (
	TARGET  Abducer
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
	COMMAND make ICE_HOME=${ICE_HOME}
)

# (2011-12-23,mmarko):
#   This target (clean-*) will be discovered and executed by CAST Control
add_custom_target (clean-Abducer
   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
   COMMAND make clean
   )

#   This target will also be executed as a part of the clean-all target defined
#   in the top-level CMakeLists.txt
if(TARGET clean-all)
   add_dependencies(clean-all DEPENDS clean-Abducer)
endif()
