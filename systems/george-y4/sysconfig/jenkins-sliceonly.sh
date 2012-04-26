#!/bin/bash

# workaround for missing "readlink -f"
cd -P -- "$(dirname -- "$0")" &&
SCRIPT=$(printf '%s\n' "$(pwd -P)/$(basename -- "$0")")

SCRIPT_DIR=$(dirname $SCRIPT)

COGX_ROOT=$SCRIPT_DIR/..
TOOLS=$COGX_ROOT/tools/scripts
BUILD_ROOT=$COGX_ROOT/BUILD
CMAKE_CACHE=$BUILD_ROOT/CMakeCache.txt

if [ ! -d $BUILD_ROOT ]; then
   mkdir $BUILD_ROOT
fi

if [ ! -d $BUILD_ROOT ]; then
   echo "BUILD directory could not be created! Can't continue."
   exit 0
fi

cd $BUILD_ROOT

# Initial cache
if [ -f $CMAKE_CACHE ]; then
   rm $CMAKE_CACHE
fi
cmake -Wno-dev $COGX_ROOT > /dev/null
$TOOLS/cmake-apply  $COGX_ROOT/BUILD  $SCRIPT_DIR/cmakecache/george-y4.txt

# Slice-only cache & build
cmakedefines="-DDO_ANT:BOOL=ON -DDO_SLICE_ONLY:BOOL=ON -DDO_SLICE_DISABLED:BOOL=ON"
cmake -Wno-dev $cmakedefines $COGX_ROOT | grep -e '--  \* Do '
make install

# Recreate initial cache
if [ -f $CMAKE_CACHE ]; then
   rm $CMAKE_CACHE
fi
cmake -Wno-dev $COGX_ROOT > /dev/null
$TOOLS/cmake-apply  $COGX_ROOT/BUILD  $SCRIPT_DIR/cmakecache/george-y4.txt
cmake -Wno-dev $COGX_ROOT | grep -e '--  \* Do '
