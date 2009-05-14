clear
echo  ===================================================
echo  Comsys.mk4 [ test1 ] 
echo  ---------------------------------------------------
echo  comsys.subarch: ASR, parsing, viz
echo
date
echo  ===================================================
echo

java -Djava.library.path=$CODE_HOME/output/lib -classpath $CODE_HOME/output/classes/:$CODE_HOME/tools/openccg/lib/ cast.server.CASTProcessServer -h localhost -f $CODE_HOME/subarchitectures/comsys.mk4/config/test-configurations/test4-SYS.cast