clear
echo  ========================================================
echo  Comsys.mk4 [ test3 ] 
echo  --------------------------------------------------------
echo  comsys.subarch: ASR, parsing, binding monitor, viz
echo  binding.subarch: binding scorers 
echo  vision.subarch: fake vision -- red thing, two blue things
echo  spatial.sa: left, right projective spatial relations
echo
date
echo  ========================================================
echo
java -Djava.library.path=$CODE_HOME/output/lib -classpath $CODE_HOME/output/classes/:$CODE_HOME/tools/openccg/lib/ cast.server.CASTProcessServer -h localhost -f $CODE_HOME/subarchitectures/comsys.mk4/config/test-configurations/test3-SYS.cast
echo ADD -g to java statement to run gui
