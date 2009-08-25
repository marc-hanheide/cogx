#!/bin/bash

# startup the CAST servers. pretty hacky for now

# where things are for runtime
CAST_DIR=.
CAST_OUTPUT_DIR=${CAST_DIR}/output
CAST_BIN_DIR=${CAST_OUTPUT_DIR}/bin
CAST_LIB_DIR=${CAST_OUTPUT_DIR}/lib
CAST_CLASSES_DIR=${CAST_OUTPUT_DIR}/classes

# ice config info
export ICE_CONFIG=${CAST_DIR}/cast_ice_config

java -ea -classpath ${CAST_CLASSES_DIR}:$CLASSPATH cast.server.ComponentServer &
JAVA_SERVER_JOB=$!


SAVED_DYLIB_PATH=${DYLD_LIBRARY_PATH}
SAVED_LIB_PATH=${LD_LIBRARY_PATH}
export DYLD_LIBRARY_PATH=${CAST_LIB_DIR}:${DYLD_LIBRARY_PATH}
export LD_LIBRARY_PATH=${CAST_LIB_DIR}:${LD_LIBRARY_PATH}
${CAST_BIN_DIR}/componentServer &
CPP_SERVER_JOB=$!

echo Java server: ${JAVA_SERVER_JOB}
echo CPP server: ${CPP_SERVER_JOB}


# if we're killed, take down everyone else with us
trap "kill  ${CPP_SERVER_JOB}; kill  ${JAVA_SERVER_JOB}" INT TERM EXIT

wait

# just in case
kill ${CPP_SERVER_JOB}
kill ${JAVA_SERVER_JOB}

# reset evn vars
export LD_LIBRARY_PATH=${SAVED_LIB_PATH}
export DYLD_LIBRARY_PATH=${SAVED_DYLIB_PATH}
unset ICE_CONFIG