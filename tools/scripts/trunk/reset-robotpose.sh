#!/bin/bash

SCRIPT_DIR=$(dirname $(readlink -f $0))

cp "$SCRIPT_DIR/default-robotpose.ccf" ./robotpose.ccf