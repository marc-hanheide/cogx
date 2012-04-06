#!/bin/bash

FILES="robotpose.ccf
       NodeGridMaps.txt
       GridMap.txt
       HeightMap.txt
       tmpmap.graph
       tmpmap.metric
       Places.txt
       rooms.xml
       place_properties.bin"

for f in $FILES ; do
	cp $f local-saved-map/
done
