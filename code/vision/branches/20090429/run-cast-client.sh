#!/bin/bash

# where things are for runtime
CAST_DIR=.
CAST_OUTPUT_DIR=${CAST_DIR}/output
CAST_BIN_DIR=${CAST_OUTPUT_DIR}/bin
CAST_LIB_DIR=${CAST_OUTPUT_DIR}/lib
CAST_CLASSES_DIR=${CAST_OUTPUT_DIR}/classes

# ice config info
export ICE_CONFIG=${CAST_DIR}/cast_ice_config

java -ea -classpath ${CAST_CLASSES_DIR}:${CLASSPATH} cast.clients.CASTClient -f $1


unset ICE_CONFIG