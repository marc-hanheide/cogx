#!/bin/bash

PIDS=""

trap 'kill $PIDS' INT TERM PIPE QUIT ABRT HUP 

LOGNAME=./AAAI-log.`date "+%F_%R"`.xml

xterm -e bash -c "tools/scripts/log4j-server log4j.properties.server > $LOGNAME" &
PIDS="$PIDS $!"
sleep 3

xterm -e player subarchitectures/nav.sa/config/stage_models/dfki_lt_objsearch/cogxp3-dfki.cfg &
PIDS="$PIDS $!"

xterm -e peekabot &
PIDS="$PIDS $!"

xterm -e output/bin/cast-server-start &
SERVERPID="$!"
PIDS="$PIDS $SERVERPID"

sleep 3

xterm -e output/bin/cast-client-start instantiations/AAAI-experimentation.cast &
PIDS="$PIDS $!"

sleep 20

build/tools/hardware/robotbase/src/c++/components/TourGuide 3.2 0
#output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority ExploreMotive NORMAL
output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority CategorizeRoomMotive LOW



wait $SERVERPID
kill $PIDS >/dev/null 2>&1