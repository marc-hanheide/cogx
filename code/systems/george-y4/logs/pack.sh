#!/bin/bash

rm logs.zip
cd ..
zip -Dj logs/logs.zip logs/*.html logs/log.xml subarchitectures/planner.sa/src/python/planner-log*.log
