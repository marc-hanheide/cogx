#!/bin/bash

PIDS=""

trap 'kill $PIDS' INT TERM PIPE QUIT ABRT HUP 

if [ "$1" ]; then
    configFile="$1"
else
    configFile="instantiations/dora-tro-real.cast"
fi

LOGNAME=./logs/log

#xterm -e bash -c "output/bin/log4j-server log4j.properties.server > $LOGNAME.xml" &
#PIDS="$PIDS $!"

xterm -e player instantiations/player/cogx-platform.cfg &
PIDS="$PIDS $!"

sleep 2
LD_LIBRARY_PATH=output/lib:/usr/local/lib/cast output/bin/display-server &
PIDS="$PIDS $!"
sleep 2


./restoremap.sh
dc1394_reset_bus

#xterm -e peekabot &
#PIDS="$PIDS $!"

xterm -e bash -c "output/bin/cast-server-start 2>&1 | tee server.log" &
SERVERPID="$!"
PIDS="$PIDS $SERVERPID"

sleep 3

xterm -e bash -c "output/bin/cast-client-start $configFile  2>&1 | tee client.log" &
#xterm -e output/bin/cast-client-start instantiations/binding-planner-test.cast &
PIDS="$PIDS $!"

#sleep 20

#BUILD/tools/hardware/robotbase/src/c++/components/TourGuide 0 0  1 0 5.8 0 5.8 4
#BUILD/tools/hardware/robotbase/src/c++/components/TourGuide 0 0  1 0 6.3 0

#if [ "$1" ]; then
#    output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority GeneralGoalMotive NORMAL
#else
#    output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority ExploreMotive NORMAL
#    output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority CategorizeRoomMotive NORMAL
#fi
#BUILD/tools/hardware/robotbase/src/c++/components/TourGuide | tee "$LOGNAME".dist.log

wait $SERVERPID
kill $PIDS >/dev/null 2>&1
