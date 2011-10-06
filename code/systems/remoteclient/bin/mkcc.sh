#!/bin/bash

bindir=`pwd`
cd ..
rootdir=`pwd`
cd $bindir
sed -e "s#@BINDIR@#$bindir#" -e "s#@ROOTDIR@#$rootdir#" $1 > $2
