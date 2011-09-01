SET(MEX_EXTENSION mexglx)
INCLUDE(FindMatlab)
FIND_PROGRAM(MEX_COMPILER mex)
IF(MEX_COMPILER)


SET(MEX_OUT "${NAO_DIR}/m")

SET(LIBS_SYS
  -lrt
  -lutil 
  -ldl
  -lboost_date_time-mt 
  -lboost_thread-mt 
  -lboost_program_options-mt 
  -lboost_serialization-mt 
  -lboost_regex-mt 
  -lboost_filesystem-mt 
  -lboost_signals-mt 
  -lboost_system-mt
  -lcv 
  -lcxcore 
  -lhighgui  )

set ( LIBS_AL
	 -L${AL_DIR}/lib
   -lalcommon
   -lalvalue
   -lippc
   -llibcore
   -lfindippc
   -llibsoap
   -ltinyxml 
)

set (INC_AL
    -I${AL_DIR}/modules/proxies
    -I${AL_DIR}/include/python2.6
    -I${AL_DIR}/include
    -I${AL_DIR}/include/alcore
    -I${AL_DIR}/include/alcommon
    -I${AL_DIR}/include/libthread
    -I${AL_DIR}/include/alvalue
    -I${AL_DIR}/include/alaudio
    -I${AL_DIR}/include/albonjourdiscovery
    -I${AL_DIR}/include/alfactory
    -I${AL_DIR}/include/allauncher
    -I${AL_DIR}/include/almath
    -I${AL_DIR}/include/alproxies
    -I${AL_DIR}/include/alsa
    -I${AL_DIR}/include/alstreamer
    -I${AL_DIR}/include/altools
    -I${AL_DIR}/include/alvision
    -I${AL_DIR}/include/tinyxml
)

set ( LIBS_AK
	-L${NAO_DIR}/lib
  -lnaobridge
)

set ( INC_AK
   -I${NAO_DIR}/src/
)

MACRO (CREATE_MEX _name)
	ADD_CUSTOM_TARGET(${_name} 
	ALL echo
	DEPENDS ${_name}.${MEX_EXTENSION}
	)
	ADD_CUSTOM_COMMAND(
	OUTPUT ${_name}.${MEX_EXTENSION} 
	COMMAND   ${MEX_COMPILER} ${CMAKE_CURRENT_SOURCE_DIR}/${_name}.cpp 
		ARGS      -O ${INC_AL} ${INC_AK} -outdir ${MEX_OUT} ${LIBS_AK} ${LIBS_AL} ${LIBS_SYS} 
		COMMENT   "MEX"
	)
ENDMACRO (CREATE_MEX _name)

CREATE_MEX(mex_naoqi)
CREATE_MEX(mex_naoqi_close)
CREATE_MEX(mex_setWalkTargetVelocity)
CREATE_MEX(mex_setStiffnesses)
CREATE_MEX(mex_stiffnessInterpolation)
CREATE_MEX(mex_setAngles)
CREATE_MEX(mex_getAngles)
CREATE_MEX(mex_getPosition)
CREATE_MEX(mex_getCameraImage)
CREATE_MEX(mex_walkTo)
CREATE_MEX(mex_walkIsActive)
CREATE_MEX(mex_say)
ENDIF(MEX_COMPILER)