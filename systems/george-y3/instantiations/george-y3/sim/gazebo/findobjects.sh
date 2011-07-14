#!/bin/bash

if [ "$1" == "" ]; then
   echo "Use: $0 <world_file>"
   echo ""
   echo "It will extract names from the lines like:"
   echo "  <model:physical name='bircher'> <!-- OBJECT -->"
   exit
fi

out="${1%.world}.objects.txt"

echo "[objects]" > $out
grep -e "<model:.*<\!-- OBJECT" $1 | sed -e "s/.*name=[\"']\([-a-zA-Z_0-9]*\)[\"']>.*$/\1/" >> $out

echo "[places]" >> $out
grep -e "<xyz>.*<\!-- PLACE" $1 | sed -e "s/.*<xyz>\(.*\)<\/xyz>.*$/\1/" >> $out


