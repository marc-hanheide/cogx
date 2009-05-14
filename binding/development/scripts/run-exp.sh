#!/bin/bash

export LD_LIBRARY_PATH=/home/nah/svn.cosy/code/output/lib 
EXP_DIR=/home/nah/svn.cosy/code/subarchitectures/benchmark.sa/config/exp/  
LOG_FILE=exp.`date +%s`
NAMING_HOST=laptop-cosy1

echo $LOG_FILE > $LOG_FILE

for EXP_CONFIG in `ls $EXP_DIR`
      do
  echo $EXP_CONFIG
  echo "" >> $LOG_FILE
  echo java -Xmx256m -classpath output/classes/ caat.server.CAATProcessServer -h $NAMING_HOST -r 120000 -f $EXP_DIR$EXP_CONFIG >> $LOG_FILE
  java -Xmx256m -classpath output/classes/ caat.server.CAATProcessServer -h $NAMING_HOST  -r 120000 -f $EXP_DIR$EXP_CONFIG >> $LOG_FILE
done