cmake_minimum_required (VERSION 2.6)

project (Abducer)

add_custom_target (Abducer ALL)
add_custom_command (
	TARGET  Abducer
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
	COMMAND make ICE_HOME=${ICE_HOME}
)

