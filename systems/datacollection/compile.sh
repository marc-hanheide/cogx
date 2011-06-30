#!/bin/bash

chmod a+x cleandata.sh
chmod a+x packdata.sh
mkdir BUILD
cd BUILD
rm CMakeCache.txt
cmake .. -C ../cmake/cache/datacollection.cmake .. 
make install

