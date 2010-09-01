#! /bin/bash

echo "Start player"
player instantiations/botx.cfg &
echo "Sleep 3"
sleep 5

echo "Start playerjoy"
playerjoy -c -dev /dev/input/js0

echo "Killing player"
killall player
