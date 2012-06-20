#!/bin/bash
# start peekabot and make the window smaller
peekabot &
sleep 1
window_id=$(wmctrl -l | grep "peekabot$" | sed "s/ .*$//");
echo $window_id
xdotool windowactivate $window_id key alt+F5  # unmaximize window
xdotool windowsize $window_id 40% 60%         # resize window
xdotool windowactivate $window_id key F9      # hide side pane

