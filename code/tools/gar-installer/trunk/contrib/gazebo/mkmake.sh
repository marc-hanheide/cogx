#!/bin/bash
mkf="Makefile"
echo '# This file was auto generated' > $mkf
#echo 'RELEASE = $(SYSTEM_NAME)-$(SYSTEM_VERSION)' >> $mkf # provided by gar.extend.mk
echo 'include ../../gar.detect.mk' >> $mkf
echo 'RELEASE = $(SYSTEM_RELEASE)' >> $mkf
echo 'default: build' >> $mkf
echo >> $mkf

TARGETS='clean fetch checksum extract patch configure build install' # Defined by GAR
VERSIONS='Ubuntu-11.04 Ubuntu-11.10 Darwin-x' # stored in RELEASE

for target in $TARGETS ; do
   echo "$target: $target-\$(RELEASE)" >> $mkf
   echo >> $mkf
done

for version in $VERSIONS; do
   if [ "$version" == "Ubuntu-11.04" ]; then
      wrkdir='gazebo-0.9'
   elif [ "$version" == "Ubuntu-11.10" ]; then
      wrkdir='gazebo-trunk'
   elif [ "$version" == "Darwin-x" ]; then
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

