#!/bin/bash

rm logs.zip
cd ..
zip logs/logs.zip logs/*.html logs/log.xml subarchitectures/planner.sa/src/python/planner-log*.log
