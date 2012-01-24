root="/home/mmarko/Documents/doc/Devel/CogX/code/systems/ul"
root="/home/mmarko/Documents/luvss/Devel/CogX/code/systems/ul"

mdlpath="$root/xdata/models/800"
mdls="CvetMetaTea ShelcoreCube SwGreenTea TwEarlGrey"

rm $mdlpath/models.db
for a in $mdls; do
   python mksqlite.py $mdlpath/models.db $mdlpath/$a
done

mdlpath="$root/xdata/models/640"
mdls="ShelcoreCube SwGreenTea TwLemonTea"

rm $mdlpath/models.db
for a in $mdls; do
   python mksqlite.py $mdlpath/models.db $mdlpath/$a
done

mdlpath="$root/xdata/models/george640"
mdls="
ColgateHerbalMineral
HagenDazsBrownie
JaffaCakesOrange
MDarjeeling
PilotBL-G2-5-Blue
PilotBL-G2-5-Red
PilotBPS-135-F-Red
ShelcoreCube2
Sony5DvdR
SwGreenTea2
TknWhiteTea
TwLemonTea2
"

rm $mdlpath/models.db
for a in $mdls; do
   python mksqlite.py $mdlpath/models.db $mdlpath/$a
done

