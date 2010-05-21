# This file is auto-generated. Do not edit
environment="""
COGX_ROOT=[PWD]

CAST_DIR=/usr/local
CAST_INSTALL_ROOT=${CAST_DIR}

CUDA_ROOT=/usr/local/cuda
CURE_LIB_DIR=/usr/local/lib/cure
MERCURY_ROOT=/usr/local/mercury-rotd-2009-10-25
MATLAB_RUNTIME_DIR=/opt/MATLAB/MATLAB_Compiler_Runtime/v78/runtime/glnx86
# MATLAB_RUNTIME_DIR=/usr/share/matlabR2008a/bin/glnx86

COGX_BUILD_DIR=${COGX_ROOT}/BUILD
COGX_LIB_DIR=${COGX_ROOT}/output/lib
COGX_PY_DIR=${COGX_ROOT}/output/python
COGX_CLASS_DIR=${COGX_ROOT}/output/classes

CAST_BIN_PREFIX=bin
CAST_BIN_DIR=${CAST_INSTALL_ROOT}/${CAST_BIN_PREFIX}

CAST_LIB_PREFIX=lib/cast
CAST_LIB_DIR=${CAST_INSTALL_ROOT}/${CAST_LIB_PREFIX}
CAST_PY_DIR=${CAST_LIB_DIR}/python

CAST_JAR=${CAST_INSTALL_ROOT}/share/java/cast.jar

CAST_CONFIG_PATH=share/cast/config/cast_ice_config
CAST_ICE_CONFIG=${CAST_INSTALL_ROOT}/${CAST_CONFIG_PATH}

ICE_CONFIG=${CAST_ICE_CONFIG}
ICE_JARS=/usr/share/java/Ice.jar:/usr/share/java/ant-ice.jar

LOG4_JAR=/usr/share/java/log4j-1.2.jar

PATH=${COGX_ROOT}/output/bin:${CUDA_ROOT}/bin:${MERCURY_ROOT}/bin:${PATH}
LD_LIB_EXTRA=${CAST_LIB_DIR}:${COGX_LIB_DIR}:${CURE_LIB_DIR}:${CUDA_ROOT}/lib:${MATLAB_RUNTIME_DIR}
LD_LIBRARY_PATH=${LD_LIB_EXTRA}:${LD_LIBRARY_PATH}
DYLD_LIBRARY_PATH=${LD_LIB_EXTRA}:${DYLD_LIBRARY_PATH}

CLASSPATH=${CLASSPATH}:${ICE_JARS}:${CAST_JAR}:${COGX_CLASS_DIR}:${LOG4_JAR}
CMD_JAVA=java -ea -classpath ${CLASSPATH}

PYTHONPATH=${PYTHONPATH}:${CAST_PY_DIR}:${COGX_PY_DIR}

CMD_CPP_SERVER=${CAST_BIN_DIR}/cast-server-c++
CMD_JAVA_SERVER=${CMD_JAVA} cast.server.ComponentServer
CMD_PYTHON_SERVER=python -m ComponentServer
CMD_CAST_CLIENT=${CMD_JAVA} cast.clients.CASTClient -f [CAST_CONFIG]
CMD_PLAYER=player [PLAYER_CONFIG]
CMD_PEEKABOT=peekabot
CMD_LOG4J_SERVER=${CMD_JAVA} org.apache.log4j.net.SimpleSocketServer [LOG4J_PORT] [LOG4J_SERVER_CONFIG]

"""
cleanup="""
#dc1394_reset_bus
#rm robotpose.ccf
#tools/scripts/peekabotdata ${COGX_ROOT}/instantiations/peekabot-models/data

"""
useroptions="""
#EDITOR=gvim --servername GVIM --remote-silent %l[+:] %s
#EDITOR=xemacs %l[+] %s
#EDITOR=kate %l[-l ] %s
#EDITOR=gedit %l[+] %s
#EDITOR=xjed %s %l[-g ]
#EDITOR=edit %l[+] %s
#EDITOR=open -a textwrangler %s

#TERMINAL=konsole --workdir '%s'
#TERMINAL=konsole --new-tab --workdir '%s'
#TERMINAL=gnome-terminal --working-directory='%s'
"""
log4joptions="""

[LOG4J.SimpleSocketServer.conf]
# Startup configuration for org.apache.log4j.net.SimpleSocketServer 
log4j.rootLogger=TRACE, console, XMLLog

[LOG4J.SimpleSocketServer.console]
log4j.appender.console=org.apache.log4j.ConsoleAppender
log4j.appender.console.Threshold=${LEVEL}
log4j.appender.console.layout=org.apache.log4j.PatternLayout
log4j.appender.console.layout.ConversionPattern=[%p %c - %m]%n

[LOG4J.SimpleSocketServer.xmlfile]
log4j.appender.XMLLog=org.apache.log4j.FileAppender
log4j.appender.XMLLog.Threshold=${LEVEL}
log4j.appender.XMLLog.File=${LOGFILE}
log4j.appender.XMLLog.Append=true
log4j.appender.XMLLog.layout=org.apache.log4j.xml.XMLLayout

# log4j.appender.console.layout=cast.core.logging.ComponentLayout
# log4j.appender.console.layout.ConversionPattern=%S %30c [ %05p ] - %m %n%E
#
#log4j.appender.console=org.apache.log4j.ConsoleAppender
#log4j.appender.console.layout=org.apache.log4j.xml.XMLLayout

[LOG4J.SimpleSocketServer.XMLLayout.head]
<?xml version="1.0" encoding="UTF-8"?>
<log4j:logsequence xmlns:log4j="http://jakarta.apache.org/log4j/" user="${USER}" time="${NOW}">
"""
