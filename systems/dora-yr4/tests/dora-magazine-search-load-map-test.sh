#!/bin/bash

#define NOVNC=1 when you call this script to run it in a regular X server
################################################################################
# define those to fit your case:
CAST_CFG="instantiations/dora-yr4-bham-jenkins-sim.cast"
GOAL="(exists (?o - visualobject) (and (= (label ?o) magazine) (kval 'ROBOT' (related-to ?o))))"
PLAYER_CFG="instantiations/stage/BHAM/cs-2-small-furniture2.cfg"
TEST_CFG="--wait 30 --wmcheck-proposedContainer \\\"sa=planner.sa;xpath=//distribs[entry/de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution/values/values/de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair/val/prop[node()='magazine']
and  entry/de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution/values/values/de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair/val/prop[node()='in']
and entry/de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution/values/values/de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair/val/pointer/subarchitecture[node()='planner.sa']];type=eu.cogx.beliefs.slice.HypotheticalBelief;qualifier=EXISTS\\\" --whcheck-containerInRoom \\\"sa=planner.sa;xpath=//distribs[entry/de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution/values/values/de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair/val/prop[node()='container'] and
 entry/de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution/values/values/de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair/val/prop[node()='in'] and entry/de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution/values/values/de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair/val/pointer/subarchitecture[node()='coma']];type=eu.cogx.beliefs.slice.HypotheticalBelief;qualifier=EXISTS\\\""
################################################################################

export NOVNC

DIR="$( cd -P "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "$DIR/.."
pwd

echo run ./run-test.sh  "$CAST_CFG"  "$GOAL"   "$PLAYER_CFG"   "$TEST_CFG"
./run-test.sh  "$CAST_CFG"  "$GOAL"   "$PLAYER_CFG"   "$TEST_CFG"
