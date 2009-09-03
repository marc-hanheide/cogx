#!/bin/bash

if [ ! -f $1.xml ]
then
    echo file $1.xml does not exist
    exit
fi

DATE=$(date "+%Y-%m-%d-%H:%M:%S")
LOG=./${1}.${DATE}.log
{
  nohup nice nnl_ndim $1.xml $1
} 2>&1 | tee ${LOG}> /dev/null &
