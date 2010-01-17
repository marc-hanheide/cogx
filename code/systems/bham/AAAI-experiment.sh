#!/bin/bash

PIDS=""

trap 'kill $PIDS' INT TERM PIPE QUIT ABRT HUP 

LOGNAME=./AAAI-log.`date "+%F_%R"`

xterm -e bash -c "tools/scripts/log4j-server log4j.properties.server > $LOGNAME.xml" &
PIDS="$PIDS $!"
sleep 3

xterm -e player subarchitectures/nav.sa/config/stage_models/dfki_lt_objsearch/cogxp3-AAAI.cfg &
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

BUILD/tools/hardware/robotbase/src/c++/components/TourGuide 0 0  1 0 5.8 0 5.8 4
#BUILD/tools/hardware/robotbase/src/c++/components/TourGuide 0 0  1 0 6.3 0

output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority ExploreMotive NORMAL
output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority CategorizeRoomMotive NORMAL

BUILD/tools/hardware/robotbase/src/c++/components/TourGuide | tee "$LOGNAME".dist.log

wait $SERVERPID
kill $PIDS >/dev/null 2>&1