#!/bin/bash

time=0;
while true; do
  last=$(stat -c %Y $1)
  #echo $last
  if [ "$last" -gt "$time" ]; then
    echo "$1 changed, creating $1.png"
    dot $1 -Tpng -o tmp_$1.png
    mv tmp_$1.png $1.png
    last=$(stat -c %Y $1)
    time=$last
  fi
  sleep 1
done;

