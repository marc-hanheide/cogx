#!/bin/sh

# OpenCCG autorun script
#
# @author: Todd Shore (Todd.Shore@dfki.de)
# @version: 2010-11-03
#
# Script to quickly run OpenCCG with a particular grammar, setting up appropriate environment variables. Due to the way OpenCCG runs with the script "tccg", this script can only be used by placing it in the directory containing the grammar you want to use, then executing this script from the terminal while currently in that directory, i.e. "./run.sh".

export JAVA_HOME="/usr/";
export OPENCCG_HOME="../../";
export PATH="$PATH:$OPENCCG_HOME/bin";

tccg
