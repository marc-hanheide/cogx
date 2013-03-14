#!/bin/bash
#zenity --question --text "reset mapping?"  && rm -f robotpose.ccf tmpmap.* && exit 0
#zenity --question --text "restore start map?" && rm -f robotpose.ccf && cp instantiations/map_backup/bham_lab/* .
rm -f robotpose.ccf tmpmap.* NodeGridMaps.txt GridMap.txt HeightMap.txt Places.txt Connectivities.txt place_properties.bin rooms.xml odom.tdf scans.tdf slaminputdata.txt conceptual.* subarchitectures/planner.sa/src/python/history-*.pddl subarchitectures/planner.sa/src/python/planner-log-*.log subarchitectures/planner.sa/src/python/standalone/tmp/static_dir_for_debugging/*
