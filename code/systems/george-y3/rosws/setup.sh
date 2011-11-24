#!/bin/sh
SCRIPT_PATH="${BASH_SOURCE[0]}"
echo $SCRIPT_PATH
rosdir=/opt/ros/electric
curdir=$(pwd)
source $rosdir/setup.bash
export ROS_ROOT=$rosdir/ros
export PATH=$ROS_ROOT/bin:$PATH
export PYTHONPATH=$ROS_ROOT/core/roslib/src:$PYTHONPATH
export ROS_PACKAGE_PATH=$curdir:$rosdir/stacks:$ROS_PACKAGE_PATH
