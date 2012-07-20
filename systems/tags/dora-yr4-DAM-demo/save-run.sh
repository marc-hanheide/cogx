#!/bin/sh
if [ "$1" ]; then
	PREFIX="$1"
else
	PREFIX="LOG"
fi
FILENAME=~/Desktop/IJCAI/"${PREFIX}_`date "+%F_%H-%M-%S"`.zip"
echo "zipping as $FILENAME"

zip -j -r "$FILENAME" SearchLog.txt logs/log.xml tmpmap.* subarchitectures/planner.sa/src/python/standalone/tmp/static_dir_for_debugging/ conceptual.*
