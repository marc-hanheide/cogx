#!/bin/bash
# automata_extraction.sh

if (($# != 4));
then
	echo 'USAGE: automata_extraction.sh <firstStartPosition> <lastStartPosition> <numberOfIterations> <experiment (offline/active)>'
	exit 1;
fi

firstStartPosition=$1;
lastStartPosition=$2;
numberOfIterations=$3;

# program="test"
if [ "$4" = "offline" ]
then
	program="/usr/local/bin/SMLearning/offline_experiment"
elif [ "$4" = "active" ]
then
	program="/usr/local/bin/SMLearning/activelearn_experiment"
fi

echo "Starting experiments for positions $firstStartPosition to $lastStartPosition.";

for ((i=$firstStartPosition;i<=$lastStartPosition;i++));
do
	nextDir="$(date +%m)$(date +%d)_${numberOfIterations}samples$4L_${i}";
	mkdir $nextDir;
	cd $nextDir;
	echo "Starting experiment sequence from position $i.";
	echo $program

	$program /usr/local/bin/SMLearning/activelearn_experiment.xml $numberOfIterations $i;	

	if (($? !=0 )); 
	then
		echo "WARNING: Experiment sequence from position $i failed, repeating it.";
		rm *;
		cd ..;
		rmdir $nextDir;
		((--i));
		continue;
	else
		echo "Experiment sequence from position $i ended successfully.";
	fi

	cd ..;
done

echo "All experiments sequences stored."


