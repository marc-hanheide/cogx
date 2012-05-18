#!/bin/bash
MAP=cs-2-small-furniture2-DAM
#zenity --question --text "reset mapping?"  && rm -f robotpose.ccf tmpmap.* && exit 0
#zenity --question --text "restore start map?" && rm -f robotpose.ccf && cp instantiations/map_backup/bham_lab/* .
./cleanmap.sh
echo -n "restore '$MAP'..."
#cp -v instantiations/map_backup/socs/tmpmap* .
#cp -v instantiations/map_backup/tro_sim/tmpmap* .
#cp -v instantiations/map_backup/socs_small/tmpmap* .
cp instantiations/map_backup/$MAP/* .
echo " done"
