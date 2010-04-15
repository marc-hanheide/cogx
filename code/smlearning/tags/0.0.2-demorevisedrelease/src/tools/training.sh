#!/bin/bash

if [ ! -f $1.config ]
then
    echo file $1.config does not exist
    exit
fi

nohup nice rnnlib -s $1.config > /dev/null 2>&1 &
