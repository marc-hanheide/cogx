#!/bin/bash

timestamp=`date +%d_%m_%Y_%H_%M`
mydir="subarchitectures/dialogue.sa/resources/pev-test-data/$timestamp"
mkdir -p $mydir

cp GBeliefHistory $mydir
mv subarchitectures/planner.sa/src/python/history-*.pddl $mydir
