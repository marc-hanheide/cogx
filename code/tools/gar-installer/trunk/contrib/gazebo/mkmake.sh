#!/bin/bash
mkf="Makefile"
echo '# This file was auto generated' > $mkf
echo 'RELEASE = $(shell lsb_release -si)-$(shell lsb_release -sr)' >> $mkf
echo 'default: build' >> $mkf
echo >> $mkf

TARGETS='clean fetch checksum extract patch configure build install'
VERSIONS='Ubuntu-11.04 Ubuntu-11.10 ' # info returned by lsb_release

for target in $TARGETS ; do
   echo "$target: $target-\$(RELEASE)" >> $mkf
   echo >> $mkf
done

for version in $VERSIONS; do
   if [ "$version" == "Ubuntu-11.04" ]; then
      wrkdir='gazebo-0.9'
   elif [ "$version" == "Ubuntu-11.10" ]; then
      wrkdir='gazebo-trunk'
   else
      echo "Invalid system version: '$version'"
      exit
   fi
   for target in $TARGETS ; do
      echo "$target-$version:" >> $mkf
      echo "	cd $wrkdir && make $target" >> $mkf
      echo >> $mkf
   done
done

#build: build-$(RELEASE)

#clean: clean-$(RELEASE)

#build-Ubuntu-11.10:
#        cd gazebo-trunk && make build

#clean-Ubuntu-11.10:
#        cd gazebo-trunk && make clean

