#!/bin/bash
#zenity --question --text "reset mapping?"  && cp instantiations/map_backup/alu_review_sim/* .
rm -f robotpose.ccf tmpmap.*
cp instantiations/map_backup/alu_review_sim/* .

