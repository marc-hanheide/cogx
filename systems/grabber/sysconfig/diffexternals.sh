#!/bin/bash
SCRIPT=$(readlink -f $0)
SCRIPT_DIR=$(dirname $SCRIPT)
COGX_ROOT=$(readlink -f $SCRIPT_DIR/..)
echo "COGX_ROOT: $COGX_ROOT"
echo ""

tmpfile=$(tempfile)

extfile=grabber.externals
if [ "$1" != "" ]; then
   extfile="$1"
fi

svn pg svn:externals $COGX_ROOT > $tmpfile
diff -U 3 --ignore-all-space --ignore-blank-lines $SCRIPT_DIR/$extfile  $tmpfile

rm $tmpfile
