#!/bin/bash

SCRIPT_DIR=$(dirname $(readlink -f $0))
export CLASSPATH="$CLASSPATH:$SCRIPT_DIR/../../output/jar/*:$SCRIPT_DIR/../../output/classes:/usr/share/java/Ice.jar"

java castutils.castextensions.IceBinToXMLConverter $*