#!/bin/bash

if [ ! -e "tools/v4r" ]; then
   exit
fi

echo "********************************************************"
echo "** BUILDING: V4R"
echo "********************************************************"

cwd=$(pwd)
if [ ! -e "tools/v4r/BUILD" ]; then
   mkdir -p "tools/v4r/BUILD"
   cd tools/v4r/BUILD
   cmake -C ../cmake-settings-cogx.txt ..
   cd $cwd
fi

cd tools/v4r/BUILD
make
cd $cwd

echo "********************************************************"
echo "** DONE BUILDING: V4R"
echo "********************************************************"
