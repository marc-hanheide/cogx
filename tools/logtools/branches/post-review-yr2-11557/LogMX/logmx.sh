#!/bin/sh

# Getting LogMX directory
LOGMX_PATH=`dirname $0`

# If you want to use a specific JRE, add its 'bin' absolute path here:
SPECIFIC_JRE_PATH=

# If your parser needs additional JARs, add their path here (absolute or relative to LogMX directory):
LOGMX_ADDITIONAL_CLASSPATH=

# If you want to use a configuration file not located in LogMX "config/" directory:
# CONFIG_FILE_PATH="-Dconfig.file=/mypath/logmx.properties"

# Setting Java command line options
LOGMX_LIB_PATH=$LOGMX_PATH/lib
LOGMX_CLASSPATH=$LOGMX_PATH/classes:$LOGMX_PATH/parsers/classes:$LOGMX_PATH/managers/classes:$LOGMX_PATH/jar/logmx.jar:$LOGMX_LIB_PATH/xtlnf.jar
LOGMX_CLASSPATH=$LOGMX_CLASSPATH:$LOGMX_LIB_PATH/jsch.jar:$LOGMX_LIB_PATH/activation.jar:$LOGMX_LIB_PATH/mailapi.jar:$LOGMX_LIB_PATH/smtp.jar:$LOGMX_LIB_PATH/jcommon-1.0.14.jar:$LOGMX_LIB_PATH/jfreechart-1.0.11.jar
LOGMX_CLASSPATH=$LOGMX_CLASSPATH:$LOGMX_ADDITIONAL_CLASSPATH
LOGMX_MAIN=com.lightysoft.logmx.LogMX
JVM_OPTIONS="-Xmx302m $CONFIG_FILE_PATH"
PATH=$SPECIFIC_JRE_PATH:$PATH

# Starting LogMX
java $JVM_OPTIONS -cp $LOGMX_CLASSPATH $LOGMX_MAIN $1
