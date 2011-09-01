
SET(AK_SHM_MEX 0 CACHE BOOL "shm mex interface" )
IF(AK_SHM_MEX )

SET(MEX_EXTENSION mexglx)
INCLUDE(FindMatlab)
FIND_PROGRAM(MEX_COMPILER mex)

IF(MEX_COMPILER)

SET(MEX_OPENCV_LIBRARIES
  -lcv 
  -lcxcore 
  -lhighgui )

SET(MEX_BOOST_LIBRARIES
  -lboost_date_time-mt 
  -lboost_thread-mt 
  -lboost_serialization-mt 
  -lboost_regex-mt 
  -lboost_filesystem-mt 
  -lboost_signals-mt 
  -lboost_system-mt )

SET(MEX_SHM_LIBRARIES
  -L${LIBRARY_OUTPUT_PATH}
  -laksharedmemhdl)

MACRO (CREATE_MEX _name)
	ADD_CUSTOM_TARGET(${_name} 
	ALL echo
	DEPENDS  ${CMAKE_CURRENT_SOURCE_DIR}/${_name}.${MEX_EXTENSION}
	)
	ADD_CUSTOM_COMMAND(
	OUTPUT    ${CMAKE_CURRENT_SOURCE_DIR}/${_name}.${MEX_EXTENSION} 
	COMMAND   ${MEX_COMPILER} ${CMAKE_CURRENT_SOURCE_DIR}/${_name}.cpp 
  ARGS      -outdir ${CMAKE_CURRENT_SOURCE_DIR} -O -I${SHAREDMEM_DIR} -I${V4R_DIR} ${MEX_OPENCV_LIBRARIES} ${MEX_BOOST_LIBRARIES} ${MEX_SHM_LIBRARIES}
	COMMENT   "MEX"
	)
ENDMACRO (CREATE_MEX _name)


CREATE_MEX (mex_shmvar )
CREATE_MEX (mex_shmlist )
ENDIF(MEX_COMPILER)

ENDIF(AK_SHM_MEX )
