#!/bin/bash

FILES="robotpose.ccf
       NodeGridMaps.txt
       GridMap.txt
       HeightMap.txt
       tmpmap.graph
       tmpmap.metric
       Places.txt
       rooms.xml
       place_properties.bin
       Connectivities.txt"

mkdir -p local-saved-map/

for f in $FILES ; do
	cp -f $f local-saved-map/
done
