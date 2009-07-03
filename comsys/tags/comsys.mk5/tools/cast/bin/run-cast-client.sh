#!/bin/bash


# sanity check
if [ $# -ne 1 ] ; then
  echo "Usage: `basename $0` <cast install prefix, e.g. /usr/local> <cast file>"
  exit $E_BADARGS
fi

# startup the CAST servers. 

# where things are for runtime
CAST_DIR=tools/cast
CAST_INSTALL_ROOT=${CAST_DIR}

CAST_BIN_PREFIX=bin
CAST_BIN_DIR=${CAST_INSTALL_ROOT}/${CAST_BIN_PREFIX}

CAST_LIB_PREFIX=lib/cast
CAST_LIB_DIR=${CAST_INSTALL_ROOT}/${CAST_LIB_PREFIX}

CAST_JAR=${CAST_INSTALL_ROOT}/share/java/cast.jar

WE_SET_ICE_CONFIG=1
CAST_CONFIG_PATH=/config/cast_ice_config
CAST_ICE_CONFIG=${CAST_INSTALL_ROOT}/${CAST_CONFIG_PATH}


# check for ice config info
if [[ "$ICE_CONFIG" ]] ; then
    echo "--------------------------------------------------------------------------"
    echo "ICE_CONFIG is already set.";
    echo "You should also include the contents of ${CAST_ICE_CONFIG} in your config." 
    echo "--------------------------------------------------------------------------"
    WE_SET_ICE_CONFIG=0
else
    export ICE_CONFIG=${CAST_ICE_CONFIG}
fi 

java -ea -classpath ${CLASSPATH}:${CAST_JAR} cast.clients.CASTClient -f $1


if (( $WE_SET_ICE_CONFIG )) ; then
    unset ICE_CONFIG
fi







