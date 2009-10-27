#!/bin/bash
COGX_ROOT=/home/mmarko/Documents/cogxcode/dfki09/systems/ul
CAST_DIR=/usr/local

COGX_BUILD_DIR=${COGX_ROOT}/BUILD
COGX_LIB_DIR=${COGX_ROOT}/output/lib
COGX_PY_DIR=${COGX_ROOT}/output/python
COGX_CLASS_DIR=${COGX_ROOT}/output/classes

CAST_INSTALL_ROOT=${CAST_DIR}
CAST_ROOT=${CAST_INSTALL_ROOT}

CAST_BIN_PREFIX=bin
CAST_BIN_DIR=${CAST_INSTALL_ROOT}/${CAST_BIN_PREFIX}

CAST_LIB_PREFIX=lib/cast
CAST_LIB_DIR=${CAST_INSTALL_ROOT}/${CAST_LIB_PREFIX}
CAST_PY_DIR=${CAST_LIB_DIR}/python

CAST_CONFIG_PATH=share/cast/config/cast_ice_config
CAST_ICE_CONFIG=${CAST_INSTALL_ROOT}/${CAST_CONFIG_PATH}

ICE_CONFIG=${CAST_ICE_CONFIG}

CAST_JAR=${CAST_INSTALL_ROOT}/share/java/cast.jar
ICE_JARS=/usr/share/java/Ice.jar:/usr/share/java/ant-ice.jar
LOG4_JAR=/usr/share/java/log4j-1.2.jar
CLASSPATH=$ICE_JARS:$LOG4_JAR:$CAST_JAR:$CLASSPATH
echo $CLASSPATH

ant all
