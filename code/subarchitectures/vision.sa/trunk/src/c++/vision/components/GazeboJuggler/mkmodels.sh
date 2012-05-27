#!/bin/bash

root=../../../../../../..
gazebo="$root/instantiations/gazebo"
#ls $gazebo
python genmodels.py

# install models
cp xdata/test1.world $gazebo
cp xdata/test1.objects.txt $gazebo
cp xdata/test1.attrs.txt $gazebo
cp -ru xdata/Media $gazebo
cp -ru xdata/models $gazebo

