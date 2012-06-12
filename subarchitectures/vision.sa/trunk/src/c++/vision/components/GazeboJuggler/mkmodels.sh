#!/bin/bash

GazeboVersion=100

root=../../../../../../..
if [ ! -d $root/instantiations ]; then
   echo "PWD=$(pwd)"
   echo "The directory $root/instantiations doesn't exist"
   exit 1
fi

python genmodels.py

if [ ! -d $root/instantiations/xdata ]; then
   mkdir $root/instantiations/xdata 
fi
if [ ! -d $root/instantiations/xdata/gazebo ]; then
   mkdir $root/instantiations/xdata/gazebo 
fi

gazebo="$root/instantiations/xdata/gazebo"
#ls $gazebo

if [ ! -d $gazebo/Media ]; then
   mkdir $gazebo/Media 
fi
if [ ! -d $gazebo/models ]; then
   mkdir $gazebo/models 
fi

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
   cp xdata/test1.objects.txt $gazebo
   cp xdata/test1.attrs.txt $gazebo
fi

