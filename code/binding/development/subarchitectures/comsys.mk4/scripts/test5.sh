clear
echo  ========================================================
echo  Comsys.mk4 [ test5 ] 
echo  --------------------------------------------------------
echo  comsys.subarch: ASR, parsing, binding monitor, viz
echo  binding.subarch: binding scorers 
echo
echo  COMSYS uses the MOLOKO v4.1 grammar
echo 
date
echo  ========================================================
echo
java -Djava.library.path=$CODE_HOME/output/lib -classpath $CODE_HOME/output/classes/:$CODE_HOME/tools/openccg/lib/ cast.server.CASTProcessServer -h localhost -f $CODE_HOME/subarchitectures/comsys.mk4/config/test-configurations/test5-SYS.cast
echo ADD -g to java statement to run gui
