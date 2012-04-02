#!/bin/bash

dirPrfx=`pwd | sed "s@.*/\(.*\)@\1@"`
zipname=logs/${USER}_${dirPrfx}_`date +"%Y-%m-%d_%H-%M-%S"`

svn info . > logs/svn-info.txt
svn st -q . >> logs/svn-info.txt

zip -j "$zipname" logs/*.xml logs/*core* logs/svn-info.txt subarchitectures/planner.sa/src/python/standalone/tmp/static_dir_for_debugging/* subarchitectures/planner.sa/src/python/history-*.pddl tmpmap.* conceptual.fg conceptual.info place_properties.bin *Map.txt Places.txt

# A string with command options
options=$@

# An array with all the arguments
arguments=($options)

# Loop index
index=0

for argument in $options
  do
    # Incrementing index
    index=`expr $index + 1`

    # The conditions
    case $argument in
      --images) zip "$zipname-images" images/*;;
      #--other) echo "key $argument value ${arguments[index]}" ;;
    esac
  done




