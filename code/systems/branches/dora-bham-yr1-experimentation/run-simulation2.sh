#!/bin/bash

PIDS=""

trap 'kill $PIDS' INT TERM PIPE QUIT ABRT HUP 

export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}/usr/lib"
echo $LD_LIBRARY_PATH

if [ "$1" ]; then
    configFile="$1"
else
    configFile="instantiations/AAAI-experimentation.cast"
fi

LOGNAME=./log

echo "Start Log4J"
xterm -T "LOG4J" -e bash -c "output/bin/log4j-server log4j.properties.client > $LOGNAME.xml" &
PIDS="$PIDS $!"
sleep 2

echo "Start Player"
xterm -T "Player" -e player subarchitectures/nav.sa/config/stage_models/dfki_lt_objsearch/cogxp3-AAAI.cfg &
PIDS="$PIDS $!"
rm -f  robotpose.ccf

echo "Start Peekabot"
xterm -T "Peeka-fookin-bot" -hold -e peekabot &
PIDS="$PIDS $!"

echo "Start Server"
xterm -T "Cast-Server" -e bash -c "output/bin/cast-server-start 2>&1 | tee server.log" &
SERVERPID="$!"
PIDS="$PIDS $SERVERPID"

sleep 3

echo "Start Client"
xterm -hold -T "Cast-Client" -e bash -c "output/bin/cast-client-start $configFile  2>&1 | tee client.log" &
PIDS="$PIDS $!"

sleep 20

#BUILD/tools/hardware/robotbase/src/c++/components/TourGuide 0 0    1 -1    2 0    0 0 # jxh
##BUILD/tools/hardware/robotbase/src/c++/components/TourGuide 0 0  1 0      1 -1    1.4 -1.8      4 -1    0 0 # ben
#BUILD/tools/hardware/robotbase/src/c++/components/TourGuide 0 0  1 0 5.8 0 5.8 4
BUILD/tools/hardware/robotbase/src/c++/components/TourGuide 0 0 6.3 0

#if [ "$1" ]; then
#    output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority GeneralGoalMotive UNSURFACE
#else
#    output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority ExploreMotive HIGH
#    output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority CategorizeRoomMotive LOW
#    output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority GeneralGoalMotive LOW
#fi
#BUILD/tools/hardware/robotbase/src/c++/components/TourGuide | tee "$LOGNAME".dist.log

wait $SERVERPID
kill $PIDS >/dev/null 2>&1
