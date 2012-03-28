#!/bin/bash

FILES="robotpose.ccf
	     tmpmap.metric
		 NodeGridMaps.txt
		 GridMap.txt
		 conceptual.info
		 conceptual.fg
		 tmpmap.graph"


for f in $FILES ; do
	cp $f saved-maps/
done
