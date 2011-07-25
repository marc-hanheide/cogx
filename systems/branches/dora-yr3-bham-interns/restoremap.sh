#!/bin/bash
#zenity --question --text "reset mapping?"  && rm -f robotpose.ccf tmpmap.* && exit 0
#zenity --question --text "restore start map?" && rm -f robotpose.ccf && cp instantiations/map_backup/bham_lab/* .
rm -f robotpose.ccf tmpmap.*
#cp -v instantiations/map_backup/socs/tmpmap* .
cp -v instantiations/map_backup/tro_sim/tmpmap* .
#cp -v instantiations/map_backup/socs_small/tmpmap* .

