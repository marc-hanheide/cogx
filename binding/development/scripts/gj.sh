clear
echo  ========================================================
echo  Comsys.mk4 interactive binding test 080725
echo  --------------------------------------------------------
echo  comsys.sa: subarch for situated dialogue processing
echo     - ASR, parsing [MOLOKO v5], visualization, 
echo	 - discourse referents
echo     - complete binding of PLFs on main binding.sa WM
echo 
echo  binding.sa: subarch for indexical interpretations
echo  motiv.sa: subarch for intentional interpretation
echo 
date
echo  ========================================================
java -Xmx2000m -ea -classpath ./output/classes/:lib/vcomsc.jar:lib/nsc.jar cast.server.CASTProcessServer -h 127.0.0.1 -f ./subarchitectures/comsys.mk4/config/test-configurations/GJ-test.cast
