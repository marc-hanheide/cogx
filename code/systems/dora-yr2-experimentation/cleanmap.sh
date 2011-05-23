#!/bin/bash
#zenity --question --text "reset mapping?"  && rm -f robotpose.ccf tmpmap.* && exit 0
#zenity --question --text "restore start map?" && rm -f robotpose.ccf && cp instantiations/map_backup/bham_lab/* .
rm -f robotpose.ccf tmpmap.* table+with+object.txt
#cp instantiations/map_backup/bham_lab/* .

#cp instantiations/map_backup/marcs_home/* .
