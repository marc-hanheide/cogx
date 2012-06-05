#!/bin/bash

GazeboVersion=100

root=../../../../../../..
gazebo="$root/instantiations/xdata/gazebo"
#ls $gazebo
python genmodels.py

# install models
if [ "$GazeboVersion" -lt "100" ]; then
   cp xdata/test1.world $gazebo
   cp xdata/test1.objects.txt $gazebo
   cp xdata/test1.attrs.txt $gazebo
   cp -ru xdata/Media $gazebo
   cp -ru xdata/models $gazebo
else
   cp -ru xdata/Media $gazebo
   cp -ru xdata/models $gazebo
   cp xdata/test1.attrs.txt $gazebo
fi

