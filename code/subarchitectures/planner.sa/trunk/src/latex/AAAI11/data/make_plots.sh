#!/bin/sh

MAPSIM=/export/home/goebelbe/src/cogx/trunk/subarchitectures/planner.sa/src/python/mapsim

FILES="dora2-easy dora2-med dora2-hard dora3-easy dora3-med dora3-hard dora4-easy dora4-med dora4-hard"
POMDP_FILES="pomdp-med pomdp-hard"

CONFIGS="baseline  dt-s20-i2000-none dt-s50-i2000-none dt-s100-i2000-none"

for i in $FILES; do
	$MAPSIM/print_res.py  $i.avg $CONFIGS -- gnu > $i.time
done

echo "zpomdp     0     0   0.466666      20.708" > pomdp-med.time
echo "zpomdp     0     0   0.3           8.6" > pomdp-hard.time

for i in $POMDP_FILES; do
	$MAPSIM/print_res.py  $i.avg $CONFIGS -- gnupomdp >> $i.time
done


gnuplot plot.gnu

./fixbb *.eps

for i in *.eps; do
    epstopdf $i
done

mv *.eps ../plots
mv *.pdf ../plots
