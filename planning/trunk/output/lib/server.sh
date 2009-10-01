#!/bin/bash

for f in  `ps xg | grep valgrind | awk '{print $1}'` ; do kill -9 $f ; done; killall -9 java; killall -9 python; cast-server 2> errors > cast_errors 
