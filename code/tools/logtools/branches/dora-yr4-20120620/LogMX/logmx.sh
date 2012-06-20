#!/bin/sh

# === LogMX start script ===
#
# Expected arguments for this script are:
#
# file1 file2 ... fileN
#    -> Open log files "file1", "file2", ... and "fileN" in LogMX
#
# --jvmopt=[MyJVMOptions]
#    -> Ignore default JVM options given in this script and use "MyJVMOptions" instead
#
# --console
#    -> Use console mode (see possible options using --help)
#
# Here are some examples:
# ./logmx.sh logs/my_log.txt
#    -> Will open file "logs/my_log.txt" in LogMX
# ./logmx.sh logs/my_log.txt logs/my_log2.txt
#    -> Will open files "logs/my_log.txt" and "logs/my_log2.txt" in LogMX
# ./logmx.sh --jvmopt=[-Xmx2G -Dconfig.file=/mypath/logmx.properties] logs/my_log.txt
#    -> Will open file "logs/my_log.txt" in LogMX and use JVM options "-Xmx2G -Dconfig.file=/mypath/logmx.properties"
#


# Getting LogMX directory
LOGMX_PATH=`dirname $0`

# If you want to use a specific JRE, add its 'bin' absolute path here:
SPECIFIC_JRE_PATH=

# If your parser needs additional JARs, add their path here (absolute or relative to LogMX directory):
LOGMX_ADDITIONAL_CLASSPATH=

# If you want to use a configuration file not located in LogMX "config/" directory:
# CONFIG_FILE_PATH="-Dconfig.file=/mypath/logmx.properties"

# If you don't want to display a splash screen, comment this line:
SPLASH_SCREEN=-splash:pics/splash_screen.png

# Disable splash screen for Console Mode
if [ "$1" == "--console" ]; then
  SPLASH_SCREEN=
fi

# Setting default JVM options
JVM_OPTIONS="-Xmx400m $SPLASH_SCREEN $CONFIG_FILE_PATH"

# Check if "--jvmopt" option is used to set JVM options
HAS_JVM_OPT=`echo "$@" | grep "\-jvmopt="`
if [ -n "$HAS_JVM_OPT" ]; then
	# Extracting parameters from command line
	LOGMX_USER_PARAMS=`echo $@ | sed 's/--jvmopt=\[.*\]//'`
	JVM_USER_PARAMS=`echo $@ | sed 's/^.*--jvmopt=\[\(.*\)\].*$/\1/'`
	
	if [ -n "$JVM_USER_PARAMS" ]; then
		# Ignoring default JVM options and use specified options instead
		JVM_OPTIONS="$JVM_USER_PARAMS"
	fi
else
	LOGMX_USER_PARAMS="$@"
fi

# Setting Java command line options
LOGMX_LIB_PATH=$LOGMX_PATH/lib
LOGMX_CLASSPATH=$LOGMX_PATH/classes:$LOGMX_PATH/parsers/classes:$LOGMX_PATH/managers/classes:$LOGMX_PATH/jar/logmx.jar
LOGMX_CLASSPATH=$LOGMX_CLASSPATH:$LOGMX_LIB_PATH/jsch.jar:$LOGMX_LIB_PATH/activation.jar:$LOGMX_LIB_PATH/mailapi.jar
LOGMX_CLASSPATH=$LOGMX_CLASSPATH:$LOGMX_LIB_PATH/smtp.jar:$LOGMX_LIB_PATH/jcommon-1.0.14.jar:$LOGMX_LIB_PATH/jfreechart-1.0.11.jar
LOGMX_CLASSPATH=$LOGMX_CLASSPATH:$LOGMX_LIB_PATH/player.jar
LOGMX_CLASSPATH=$LOGMX_CLASSPATH:$LOGMX_ADDITIONAL_CLASSPATH
LOGMX_MAIN=com.lightysoft.logmx.LogMX
PATH=$SPECIFIC_JRE_PATH:$PATH

# Starting LogMX
java $JVM_OPTIONS -cp $LOGMX_CLASSPATH $LOGMX_MAIN $LOGMX_USER_PARAMS
