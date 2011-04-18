#!/bin/bash
SCRIPT=$(readlink -f $0)
SCRIPT_DIR=$(dirname $SCRIPT)
COGX_ROOT=$(readlink -f $SCRIPT_DIR/..)
TOOLS=$COGX_ROOT/tools/scripts

if [ -f $COGX_ROOT/BUILD/CMakeCache.txt ]; then
   echo "BUILD/CMakeCache.txt exsits! Won't continue."
   exit 0
fi

if [ ! -d $COGX_ROOT/BUILD ]; then
   mkdir $COGX_ROOT/BUILD
fi

if [ ! -d $COGX_ROOT/BUILD ]; then
   echo "BUILD directory could not be created! Can't continue."
   exit 0
fi

cd $COGX_ROOT/BUILD

cmake -Wno-dev $COGX_ROOT > /dev/null
$TOOLS/cmake-apply  $COGX_ROOT/BUILD  $SCRIPT_DIR/cmakecache/george-y3.txt
cmake -Wno-dev $COGX_ROOT > /dev/null

