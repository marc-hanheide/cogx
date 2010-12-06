#!/bin/bash
# ensemble_experiment.sh

if (($# != 5));
then
	echo 'USAGE: ensemble_experiment.sh firstStartPosition lastStartPosition numberOfIterations experiment (offline/active) storeLabels (0 -> no / 1 -> yes)'
	exit 1;
fi

firstStartPosition=$1;
lastStartPosition=$2;
numberOfIterations=$3;
storeLabels=$5;

if [ "$4" = "offline" ]
then
	program="/usr/local/bin/SMLearning/offline_experiment"
	xml="/usr/local/bin/SMLearning/offline_experiment.xml"
elif [ "$4" = "active" ]
then
	program="/usr/local/bin/SMLearning/activelearn_experiment"
	xml="/usr/local/bin/SMLearning/activelearn_experiment.xml"
fi

echo "Starting experiments for positions $firstStartPosition to $lastStartPosition.";

for ((i=$firstStartPosition;i<=$lastStartPosition;i++));
do
	nextDir="$(date +%m)$(date +%d)_${numberOfIterations}samples_$4L_${i}";
	echo "Creating directory ${nextDir}..."
	mkdir $nextDir;
	cd $nextDir;
	echo "Starting experiment sequence from position $i.";
	echo $program

	$program $xml $numberOfIterations $i $storeLabels;

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


