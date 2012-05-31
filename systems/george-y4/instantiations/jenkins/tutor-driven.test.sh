#!/bin/bash

# Prepare Gazebo paths
root=$(pwd)
cd instantiations/jenkins/data/gazebo
python $root/tools/scripts/gazebo-install-path
cd $root

#root="."
runner="python tools/castctrl/castrunner.py"

abducer="--abducer"
#cleanup="--cleanup-script=svnst.sh"
#develop="--build --rsync"
display="--display-server"
gazebo="--gazebo-world=$root/instantiations/jenkins/data/gazebo/test1.world"
#golem="--golem=dummy"
#peekabot="--peekabot"
player="--player=$root/instantiations/player/cogx-platform-sim.cfg"
#speech="--text2speech"
system="--cast-file=$root/instantiations/george/run-george-full-eval.cast"

$runner $develop $player $gazebo $golem $cleanup $peekabot $abducer $display $speech $system
