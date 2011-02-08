#!/bin/bash

### cheap hack to get opencv1.1 to work -- link to 2.1
#sudo ln -s libcv.so libcv.so.1.1
#sudo ln -s libcv.so libcv.so.1.1export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
#sudo ln -s libhighgui.so libhighgui.so.1.1
#sudo ln -s libcvaux.so libcvaux.so.1.1
#sudo ln -s libml.so libml.so.1.1

# clean up
rm -f tmpmap.graph
rm -f tmp.tmptmp

#Defaults
PEEKABOT=true
configFile="instantiations/AAAI-experimentation.cast"

# Check for options
for i in $*
do
	case $i in
#    	--searchpath=*)
#		SEARCHPATH=`echo $i | sed 's/[-a-zA-Z0-9]*=//'`
	#		;;
   	--config=*)
		echo "changed configFile"
		configFile=`echo $i | sed 's/[-a-zA-Z0-9]*=//'`
		;;
    	--peekabot)
		PEEKABOT=true
		;;
    	--no-peekabot)
		PEEKABOT=false
		;;
    	*)
                # unknown option
		;;
  	esac
done




PIDS=""

trap ' kill $SERVERPID ; echo \"killed server\" ' SIGINT INT TERM HUP ABRT QUIT 

#export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}/usr/lib"
#echo $LD_LIBRARY_PATH

LOGNAME=./log

echo "Reset Camera Bus"
dc1394_reset_bus

camerareturn=$?

if [[ $camerareturn != 0 ]]; then
	echo "Camera setup failed"
	exit 3
fi

echo "Start Log4J"
xterm -T "LOG4J" -e bash -c "output/bin/log4j-server log4j.properties.server > $LOGNAME.xml" &
PIDS="$PIDS $!"
sleep 3

echo "Start Player"
#xterm -T "Player" -e 
player instantiations/botx.cfg &
PLAYER_PID="$!"
rm -f  robotpose.ccf
sleep 5

if $PEEKABOT; then
	echo "Start Peekabot"
	xterm -T "Peekabot" -hold -e peekabot &
	PIDS="$PIDS $!"
else
	echo "Not Starting Peekabot"
fi
sleep 3

echo "Start Server"
xterm -T "Cast-Server" -e bash -c "output/bin/cast-server-start 2>&1 | tee server.log" &
SERVERPID="$!"
PIDS="$PIDS $SERVERPID"

#sleep 15
sleep 3

echo "Start Client"
echo "ConfigFile = " $configFile
xterm -hold -T "Cast-Client" -e bash -c "output/bin/cast-client-start $configFile  2>&1 | tee client.log" &
PIDS="$PIDS $!"

#echo "sleep 5 more"
#sleep 5

# #                                                                                        #####
#BUILD/tools/hardware/robotbase/src/c++/components/TourGuide 0 0  1.5 0 2 1.5 1.5 0.3 7.8 0 9.6 1 9.3 4.1
#BUILD/tools/hardware/robotbase/src/c++/components/TourGuide 0 0    1 -1    2 0    0 0 # jxh
##BUILD/tools/hardware/robotbase/src/c++/components/TourGuide 0 0  1 0      1 -1    1.4 -1.8      4 -1    0 0 # ben
#BUILD/tools/hardware/robotbase/src/c++/components/TourGuide 0 0  1 0 5.8 0 5.8 4
#BUILD/tools/hardware/robotbase/src/c++/components/TourGuide 0 0 6.3 0

#if [ "$1" ]; then
#    output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority GeneralGoalMotive UNSURFACE
#else
#    output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority ExploreMotive HIGH
#    output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority CategorizeRoomMotive LOW
#    output/bin/universalIceClient MotiveFilterManager localhost motivation.slice.RemoteFilterServer setPriority GeneralGoalMotive LOW
#fi
#BUILD/tools/hardware/robotbase/src/c++/components/TourGuide | tee "$LOGNAME".dist.log

echo "waiting for server to terminate"
wait $SERVERPID

echo "kill player"
kill -SIGINT $PLAYER_PID #>/dev/null 2>&1
kill $PIDS >/dev/null 2>&1
