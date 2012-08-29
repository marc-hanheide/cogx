#!/bin/sh

./tools/abducer/bin/abducer-server -n "AbducerServer" -e "default -p 9100" -l ./log4j.properties -a ./tools/abducer/bin/abducer-engine-pb -x --silent &
ABDUCER_JOB=$!

xterm -e bash -c "python tools/castctrl/castagent.py" &
AGENT_JOB=$!

player instantiations/player/cogx-platform.cfg &
PLAYER_JOB=$!

trap "kill ${PLAYER_JOB} ${ABDUCER_JOB} ${AGENT_JOB}; sleep 2; kill -q -9 ${PLAYER_JOB} ${ABDUCER_JOB} ${AGENT_JOB} 2>/dev/null" INT TERM EXIT

dc1394_reset_bus


wait

