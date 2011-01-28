#!/bin/sh

FILES="dora1-easy dora1-med dora1-hard dora2-easy dora2-med dora2-hard dora3-easy dora3-med dora3-hard dora4-easy dora4-med dora4-hard dora5 dora6"

CONFIGS="baseline  dt-s20-i1000-r10 dt-s50-i1000-r10 dt-s100-i2000-r10 dt-s200-i3000-r10"

for i in $FILES; do
	./extract.py  $i.avg $CONFIGS > $i.time
done
