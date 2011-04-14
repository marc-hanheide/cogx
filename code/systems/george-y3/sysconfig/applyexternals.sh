#!/bin/bash
SCRIPT=$(readlink -f $0)
SCRIPT_DIR=$(dirname $SCRIPT)
COGX_ROOT=$(readlink -f $SCRIPT_DIR/..)
echo "COGX_ROOT: $COGX_ROOT"
echo ""

svn ps svn:externals --file $SCRIPT_DIR/externals.txt $COGX_ROOT

