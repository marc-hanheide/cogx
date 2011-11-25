#!/bin/bash



DIR="$( cd -P "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "$DIR"



JARS=`find "$DIR/output/jar" -name "*.jar" | tr "\n" ":"`

export CLASSPATH=$CLASSPATH:/usr/local/share/java/cast.jar:/usr/share/java/Ice.jar:/usr/share/java/log4j-1.2.jar:$JARS
echo $CLASSPATH

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$DIR/output/lib:/usr/local/lib/cast"

PIDS=""

trap 'kill $PIDS; sleep 2; kill -9 $PIDS; exit 1' INT TERM PIPE QUIT ABRT HUP 

if [ "$1" ]; then
    configFile="$1"
else
    configFile="instantiations/dora-test-search-base.cast"
fi

mkdir -p logs
rm -f logs/log4j.properties
cat > logs/log4j.properties <<EOF
log4j.rootLogger=DEBUG,srvXmlFile,srvConsole
log4j.appender.srvXmlFile=org.apache.log4j.FileAppender
log4j.appender.srvXmlFile.Threshold=DEBUG
log4j.appender.srvXmlFile.File=log.xml
log4j.appender.srvXmlFile.Append=true
log4j.appender.srvXmlFile.layout=org.apache.log4j.xml.XMLLayout


log4j.appender.srvConsole=org.apache.log4j.ConsoleAppender
log4j.appender.srvConsole.Threshold=INFO
log4j.appender.srvConsole.layout=org.apache.log4j.PatternLayout
log4j.appender.srvConsole.layout.ConversionPattern=[%p %c: %m]%n
EOF

rm -f log4j.properties
cat > log4j.properties <<EOF
log4j.rootLogger=TRACE,cliSocketApp
#+ Section: LOG4J.client.IceAppender
#+ The clients are configured to send messages to an IceAppender (CAST logger)
log4j.loggerFactory=cast.core.logging.ComponentLoggerFactory
log4j.appender.cliSocketApp=cast.core.logging.IceAppender
log4j.appender.cliSocketApp.Host=localhost
EOF

xterm -title "log server" -e bash -c "cd logs; cast-log-server" &
PIDS="$PIDS $!"
sleep 2

xterm -e player instantiations/stage/alu/alu.cfg &
PIDS="$PIDS $!"

rm -f  robotpose.ccf tmpmap.*

#xterm -e peekabot &
#PIDS="$PIDS $!"

xterm -title "CAST server" -e bash -c "output/bin/cast-server-start 2>&1 | tee logs/server.log" &
SERVERPID="$!"
PIDS="$PIDS $SERVERPID"

xterm -title "Display server" -e bash -c "output/bin/display-server 2>&1 | tee logs/display-server.log" &
DISPLAYSERVERPID="$!"
PIDS="$PIDS $DISPLAYSERVERPID"


xterm -title "Abducer" -e bash -c "tools/abducer/bin/abducer-server -n AbducerServer -e \"default -p 9100\" -l \"$DIR/log4j.properties\" -a \"$DIR/tools/abducer/bin/abducer-engine-pb\" -x --silent 2>&1 | tee logs/abducer.log" &
ABDUCERPID="$!"
PIDS="$PIDS $ABDUCERPID"

sleep 5

xterm -title "CAST cient: $configFile" -e bash -c "output/bin/cast-client-start $configFile  2>&1 | tee logs/client.log" &
PIDS="$PIDS $!"

vncsnapshot -passwd /var/lib/jenkins/.vnc/passwd $DISPLAY 1st-shot.jpg 

sleep 20

#BUILD/tools/hardware/robotbase/src/c++/components/TourGuide 0 0  1 0 5.8 0 5.8 4
#BUILD/tools/hardware/robotbase/src/c++/components/TourGuide 0 0  1 0 6.3 0

#if [ "$1" ]; then
#    output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority GeneralGoalMotive NORMAL
#else
#    output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority ExploreMotive NORMAL
#    output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority CategorizeRoomMotive NORMAL
#fi
#BUILD/tools/hardware/robotbase/src/c++/components/TourGuide | tee "$LOGNAME".dist.log

#wait $SERVERPID
kill $PIDS >/dev/null 2>&1
sleep 2; 
kill -9 $PIDS  >/dev/null 2>&1
exit 0


