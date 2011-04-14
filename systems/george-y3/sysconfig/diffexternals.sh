#!/bin/bash
SCRIPT=$(readlink -f $0)
SCRIPT_DIR=$(dirname $SCRIPT)
COGX_ROOT=$(readlink -f $SCRIPT_DIR/..)
echo "COGX_ROOT: $COGX_ROOT"
echo ""

tmpfile=$(tempfile)

svn pg svn:externals $COGX_ROOT > $tmpfile
diff -U 3 --ignore-all-space --ignore-blank-lines $SCRIPT_DIR/externals.txt $tmpfile

rm $tmpfile
