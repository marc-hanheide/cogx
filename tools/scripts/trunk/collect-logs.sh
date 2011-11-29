#!/bin/bash


dirPrfx=`pwd | sed "s@.*/\(.*\)@\1@"`
zipname=${dirPrfx}_`date +"%Y-%m-%d_%H-%M-%S"`

zip -j "$zipname" logs/log.xml subarchitectures/planner.sa/src/python/standalone/tmp/static_dir_for_debugging/* tmpmap.* conceptual.fg conceptual.info

