#!/bin/bash

PEEKABOT_CRASH_COUNT=0
TEST_COMPLETE=0

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

if [ "$3" ]; then
    stageFile="$3"
else
    stageFile="instantiations/stage/BHAM/cs-2-small-furniture.cfg"
fi

DIR="$( cd -P "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "$DIR"

JARS=`find "$DIR/output/jar" -name "*.jar" | tr "\n" ":"`

export CLASSPATH=$CLASSPATH:/usr/local/share/java/cast.jar:/usr/share/java/Ice.jar:/usr/share/java/log4j-1.2.jar:/opt/local/share/java/cast.jar:/opt/local/share/java/Ice.jar:$JARS
echo $CLASSPATH

export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$DIR/output/lib:/usr/local/lib/cast:/opt/local/lib/cast"
export DYLD_LIBRARY_PATH="$LD_LIBRARY_PATH"

mkdir -p peekabot_crash_logs
# clear old peekabot_crash_logs
rm -rf peekabot_crash_logs/*

# if trying again after a peekabot crash, start here
function doTest {

trap 'kill -2 $PIDS; sleep 5; kill -9 $PIDS; exit 1' INT TERM PIPE QUIT ABRT HUP 

PIDS=""

mkdir -p logs
# clear old logs
rm -rf logs/*

# configure logging
cat > logs/log4j.properties <<EOF
log4j.rootLogger=TRACE,srvXmlFile,srvConsole
log4j.appender.srvXmlFile=org.apache.log4j.FileAppender
log4j.appender.srvXmlFile.Threshold=TRACE
log4j.appender.srvXmlFile.File=log.xml
log4j.appender.srvXmlFile.Append=true
log4j.appender.srvXmlFile.layout=org.apache.log4j.xml.XMLLayout

log4j.appender.srvConsole=org.apache.log4j.ConsoleAppender
log4j.appender.srvConsole.Threshold=INFO
log4j.appender.srvConsole.layout=org.apache.log4j.PatternLayout
log4j.appender.srvConsole.layout.ConversionPattern=[%p %c: %m]%n
EOF

# log coma at TRACE, everything else at DEBUG
rm -f log4j.properties
cat > log4j.properties <<EOF
log4j.rootLogger=DEBUG,cliSocketApp
#+ Section: LOG4J.client.IceAppender
#+ The clients are configured to send messages to an IceAppender (CAST logger)
log4j.loggerFactory=cast.core.logging.ComponentLoggerFactory
log4j.appender.cliSocketApp=cast.core.logging.IceAppender
log4j.appender.cliSocketApp.Host=localhost

log4j.logger.coma=TRACE
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

xterm -e player $stageFile &
PIDS="$PIDS $!"

rm -f core

echo "--------------------------"

ln -fs ~/.peekabot .

echo "starting peekabot"
export XAUTHORITY=~/.Xauthority
xauth -v exit

xterm -title "peekabot-xterm" -e "ulimit -c unlimited; /opt/VirtualGL/bin/vglrun +v -c proxy /usr/local/bin/peekabot 2>&1 | tee logs/peekabot.log" &
PIDS="$PIDS $!"

sleep 10
wmctrl -l | grep "peekabot$"

window_id=$(wmctrl -l | grep "peekabot$" | sed "s/ .*$//");
echo "peekabot window id is " $window_id
xdotool windowactivate $window_id key alt+F5
xdotool windowactivate $window_id key F9
xdotool windowsize $window_id 40% 40%

sleep 2

echo "--------------------------"
echo "starting PBDisplayControl"

xterm -title "PBDisplayControl" -e bash -c "sleep 5; output/bin/PBDisplayControl --exec \"Top down\" 2>&1 | tee logs/PBDisplayControl.log" &
PIDS="$PIDS $!"
sleep 7
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
# (Wait for 100 seconds to ensure everything has loaded)
echo "--------------------------"
echo "Sleeping for 20 secs"
sleep 20

TESTREST=0
PEEKABOT_CRASHED=0
window_id=$(wmctrl -l | grep "peekabot$" | sed "s/ .*$//");
if [ "$window_id" ]; then
	echo "Sleeping for another 80 secs"
	sleep 80
	window_id=$(wmctrl -l | grep "peekabot$" | sed "s/ .*$//");
else
	echo "peekabot has crashed already"
fi

if [ "$window_id" ]; then
	echo "peekabot window id is " $window_id
	echo "Moving Peekabot"
	xdotool windowactivate $window_id
	xdotool windowmove $window_id 0 0
	xdotool windowsize $window_id 70% 70%
	echo "Done moving Peekabot"
	sleep 2
	if [ "$GOAL" ]; then
        	echo "running test for goal $GOAL" 
		if ant -Dtest.goal="$GOAL" goaltest; then TESTREST=0; else TESTREST=1; fi
        	echo "test returned"
	fi
	TEST_COMPLETE=1
	storeCoreDump
else
	echo "Peekabot has crashed"
	PEEKABOT_CRASHED=1
	PEEKABOT_CRASH_COUNT=$(($PEEKABOT_CRASH_COUNT + 1))	
fi

# check if the C++ server is still running after this time!
if ps ax | grep  cast-server-c++ | grep -qv "grep"; then RES=$TESTREST; else RES=1; fi


kill -2 $PIDS >/dev/null 2>&1
sleep 5; 
kill -9 $PIDS  >/dev/null 2>&1

# if peekabot crashed then store some of the files somewhere else before trying again
# otherwise collect logs as normal
if [ $PEEKABOT_CRASHED -eq 1 ]; then
	mv logs/peekabot.log peekabot_crash_logs/peekabot-crash"$PEEKABOT_CRASH_COUNT".log
	# only collect the peekabot core and cast log files the first time
	if [ $PEEKABOT_CRASH_COUNT -eq 1 ]; then
		if [ -e "core" ]; then
			zip peekabot_crash_logs/pb-crash"$PEEKABOT_CRASH_COUNT"-core.zip core
		fi	
		mv logs/log.xml logs/pb-crash"$PEEKABOT_CRASH_COUNT"-cast-log.xml
		zip peekabot_crash_logs/pb-crash"$PEEKABOT_CRASH_COUNT"-cast-log.zip logs/pb-crash"$PEEKABOT_CRASH_COUNT"-cast-log.xml
	fi
else
	tools/scripts/collect-logs.sh
fi

} # end of doTest function

while [ $TEST_COMPLETE -eq 0 ] && [ $PEEKABOT_CRASH_COUNT -ge 0 ] && [ $PEEKABOT_CRASH_COUNT -lt 5 ]; do
	echo "Running test after $PEEKABOT_CRASH_COUNT peekabot crashes"
	doTest
	sleep 10
done

if [ $PEEKABOT_CRASH_COUNT -gt 0 ]; then
	# move peekabot crash logs to enable archiving, alternatively add peekabot_crash_logs/* to archive list in jenkins configuration
	mv peekabot_crash_logs/* logs/
fi

exit $RES


