#! /usr/bin/env bash
echo building planner executables
THIS_DIR=$PWD
OUTPUT_DIR=../../output/
SRC_DIR=$THIS_DIR/src/python
BASE_PLANNER_DIR=$SRC_DIR/standalone/base_planners
FF_DIR=$BASE_PLANNER_DIR/ContinualAxFF
echo create python stubs from slice files
pushd $SRC_DIR
./build_slice.sh
pushd
echo compile base planners
pushd $FF_DIR
make
pushd
echo make python code available
pushd $OUTPUT_DIR
ln -s $SRC_DIR   # $OUTPUT_DIR/python should exist now and must be added to the PYTHONPATH, eg.,  in .bashrc
pushd
echo clean up
# nothing currently
