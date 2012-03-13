#!/bin/bash


function storeCoreDump {
	if [ -e "core" ]; then
		zip logs/core.zip core
	else
		touch logs/no-core-dump
	fi
}


if [ "$1" ]; then
    configFile="$1"
else
    configFile="instantiations/dora-test-search-sim-peekabot.cast"
fi

if [ "$2" ]; then
    GOAL="$2"
else
    GOAL=""
fi

DIR="$( cd -P "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "$DIR"

JARS=`find "$DIR/output/jar" -name "*.jar" | tr "\n" ":"`

export CLASSPATH=$CLASSPATH:/usr/local/share/java/cast.jar:/usr/share/java/Ice.jar:/usr/share/java/log4j-1.2.jar:/opt/local/share/java/cast.jar:/opt/local/share/java/Ice.jar:$JARS
echo $CLASSPATH

export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$DIR/output/lib:/usr/local/lib/cast:/opt/local/lib/cast"
export DYLD_LIBRARY_PATH="$LD_LIBRARY_PATH"

trap 'kill -2 $PIDS; sleep 5; kill -9 $PIDS; exit 1' INT TERM PIPE QUIT ABRT HUP 

PIDS=""

mkdir -p logs
# clear old logs
rm -rf logs/*
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


#if which vncsnapshot; then
#	xterm -title "make snapshots" -e bash -c "vncsnapshot -fps 2 -count 10000 -passwd $HOME/.vnc/passwd $DISPLAY logs/snapshot.jpg" &
#	PIDS="$PIDS $!"
#else
#	echo "snapshot is not possible, install vncsnapshot to make it work" >& 2
#fi

#DIM=`xdpyinfo | grep dimension | cut -f7 -d" "`
#ffmpeg -f x11grab -s $DIM  -r 1  -i $DISPLAY logs/screencast.mov 2>&1  &

# Make a flash video
flvrec.py -o logs/screencast.flv -K 15 -P ~/.vnc/passwd.decrypt -r 1 $DISPLAY 2>&1 &
PIDS="$PIDS $!"

xterm -title "log server" -e bash -c "cd logs; cast-log-server" &
PIDS="$PIDS $!"
sleep 2

xterm -e player instantiations/stage/BHAM/cs-2-small.cfg &
PIDS="$PIDS $!"

rm -f  robotpose.ccf tmpmap.*
rm -f core
echo "--------------------------"

ln -fs ~/.peekabot .

echo "starting peekabot"

xterm -title "peekabot-xterm" -e "/opt/VirtualGL/bin/vglrun +v -c proxy /usr/local/bin/peekabot 2>&1 | tee logs/peekabot.log" &
PIDS="$PIDS $!"

sleep 10
wmctrl -l | grep "peekabot$"

window_id=$(wmctrl -l | grep "peekabot$" | sed "s/ .*$//");
echo "peekabot window id is " $window_id
xdotool windowactivate $window_id key alt+F5
xdotool windowsize $window_id 40% 40%

sleep 2

echo "--------------------------"
echo "starting PBDisplayControl"

xterm -title "PBDisplayControl" -e bash -c "sleep 5; output/bin/PBDisplayControl --exec \"Top down\" 2>&1 | tee logs/PBDisplayControl.log" &
PIDS="$PIDS $!"
echo "--------------------------"

xterm -title "CAST server" -e bash -c "ulimit -c unlimited; output/bin/cast-server-start 2>&1 | tee logs/server.log" &
SERVERPID="$!"
PIDS="$PIDS $SERVERPID"

xterm -title "Display server" -e bash -c "ulimit -c unlimited; output/bin/display-server 2>&1 | tee logs/display-server.log" &
DISPLAYSERVERPID="$!"
PIDS="$PIDS $DISPLAYSERVERPID"


xterm -title "Abducer" -e bash -c "ulimit -c unlimited; tools/abducer/bin/abducer-server -n AbducerServer -e \"default -p 9100\" -l \"$DIR/log4j.properties\" -a \"$DIR/tools/abducer/bin/abducer-engine-pb\" -x --silent 2>&1 | tee logs/abducer.log" &
ABDUCERPID="$!"
PIDS="$PIDS $ABDUCERPID"

sleep 10

xterm -title "CAST client: $configFile" -e bash -c "ulimit -c unlimited; output/bin/cast-client-start $configFile  2>&1 | tee logs/client.log" &
PIDS="$PIDS $!"

# in the future we will wait for the junit result here... for now, let's run the system for 60 seconds
#waitForTrigger
echo "--------------------------"
echo "Sleeping for 100 secs"

sleep 100
echo "Moving Peekabot"
xdotool windowmove $window_id 0 0
xdotool windowsize $window_id 70% 70%
xdotool windowactivate $window_id
echo "Done moving Peekabot"
sleep 2

TESTREST=0
if [ "$GOAL" ]; then
        echo "running test for goal $GOAL" 
	if ant -Dtest.goal="$GOAL" goaltest; then TESTREST=0; else TESTREST=1; fi
        echo "test returned"
fi

storeCoreDump

# check if the C++ server is still running after this time!
if ps ax | grep  cast-server-c++ | grep -qv "grep"; then RES=$TESTREST; else RES=1; fi


kill -2 $PIDS >/dev/null 2>&1
sleep 5; 
kill -9 $PIDS  >/dev/null 2>&1

tools/scripts/collect-logs.sh

exit $RES


