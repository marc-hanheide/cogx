#!/bin/bash

timestamp=`date +%Y-%m-%d_%H:%M`
mydir="subarchitectures/dialogue.sa/resources/pev-test-data/$timestamp"
mkdir -p $mydir

cp GBeliefHistory.xml $mydir
cp PlanVerbMessages.xml $mydir
cp PEV-rawtext.xml $mydir
mv subarchitectures/planner.sa/src/python/history-*.pddl $mydir
cp logs/log.xml $mydir
