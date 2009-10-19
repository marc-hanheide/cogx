cmake_minimum_required (VERSION 2.6)
project (Comsys)

get_filename_component (COMSYS_ROOT . ABSOLUTE)

find_package(Mercury REQUIRED)

# build the abducer
add_custom_target (
	abducer ALL
	make MERCURY_HOME=${MERCURY_HOME} ICE_HOME=${ICE_HOME}
	WORKING_DIRECTORY ${COMSYS_ROOT}/src/mercury-c++/abduction
)
