#!/bin/bash
SCRIPT=$(readlink -f $0)
SCRIPT_DIR=$(dirname $SCRIPT)
COGX_ROOT=$(readlink -f $SCRIPT_DIR/..)
echo "COGX_ROOT: $COGX_ROOT"
echo ""

extfile=grabber.externals
if [ "$1" != "" ]; then
   extfile="$1"
fi

svn ps svn:externals --file $SCRIPT_DIR/$extfile  $COGX_ROOT

