#!/bin/bash

#define NOVNC=1 when you call this script to run it in a regular X server
################################################################################
# define those to fit your case:
CAST_CFG="instantiations/dora-yr4-bham-jenkins-sim.cast"
GOAL="(forall (?p - place) (= (placestatus ?p) trueplace))"
PLAYER_CFG="instantiations/stage/BHAM/cs-2-small-furniture2.cfg"
TEST_CFG="--wmcheck-allplacesexplored \\\"sa=spatial.sa;xpath=//status[node()='TRUEPLACE'];type=SpatialData.Place;qualifier=ALL\\\""
################################################################################

export NOVNC

DIR="$( cd -P "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "$DIR/.."
pwd

echo run ./run-test.sh  "$CAST_CFG"  "$GOAL"   "$PLAYER_CFG"   "$TEST_CFG"
./run-test.sh  "$CAST_CFG"  "$GOAL"   "$PLAYER_CFG"   "$TEST_CFG"
