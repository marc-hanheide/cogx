#!/bin/bash
# start peekabot and make the window smaller
peekabot &
sleep 1
window_id=$(wmctrl -l | grep "peekabot$" | sed "s/ .*$//");
echo $window_id
xdotool windowactivate $window_id key alt+F5
xdotool windowsize $window_id 40% 60%

