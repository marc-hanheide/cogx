#!/bin/bash

if [ ! -d xdata ]; then
   mkdir xdata
fi

while [ 0 -lt 1 ]; do
   for f in data/* ; do
      echo $f
      cp $f xdata/
      sleep 1s
   done
   echo "--"
done
