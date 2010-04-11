MACRO( BUILD_ALCHEMY_MACRO )
	message(STATUS "Building Alchemy library")
	if(APPLE)
		exec_program(make "${COGX_ROOT}/tools/alchemy/src"
			ARGS -f makefile_MAC
			OUTPUT_VARIABLE ALCHEMY_MAKE_OUT
			RETURN_VALUE ALCHEMY_SUCCESS
		)
	else()
		exec_program(make "${COGX_ROOT}/tools/alchemy/src"
			OUTPUT_VARIABLE ALCHEMY_MAKE_OUT
			RETURN_VALUE ALCHEMY_SUCCESS
		)
	endif()

	if(NOT ALCHEMY_SUCCESS EQUAL 0)
		message(STATUS "Building Alchemy library failed")
		message(FATAL_ERROR ${ALCHEMY_MAKE_OUT})
	else()
		message(STATUS "Building Alchemy library done")
	endif()   
ENDMACRO( BUILD_ALCHEMY_MACRO )