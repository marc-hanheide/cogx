cmake_minimum_required (VERSION 2.6)
project (Dialogue)

# auto generated code
add_subdirectory (apis/cast/)
add_subdirectory (apis/dialogue/)

option (BUILD_ABDUCER "Build the abducer" NO)

if (BUILD_ABDUCER)
	configure_file(${PROJECT_SOURCE_DIR}/build-abducer.sh.in ${PROJECT_SOURCE_DIR}/build-abducer.sh)

	ADD_CUSTOM_TARGET( Abducer ALL DEPENDS ./build-abducer.sh)
	ADD_CUSTOM_COMMAND(
		TARGET  Abducer
		WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
		COMMAND bash ./build-abducer.sh
	)
endif (BUILD_ABDUCER)
