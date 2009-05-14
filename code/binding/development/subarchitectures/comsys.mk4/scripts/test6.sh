clear
echo  ========================================================
echo  Comsys.mk4 [ test6 ] 
echo  --------------------------------------------------------
echo  comsys.sa: subarch for situated dialogue processing
echo     - ASR, parsing [MOLOKO v4.1], visualization, 
echo     - incremental binding of PLFs on virtual binding.sa WM
echo     - complete [lvl 2] binding of PLFs on main binding.sa WM
echo 
echo  binding.sa: subarch for complete, stable interpretations
echo     -binding scorers 
echo 
echo  virtualbinding: subarch for interpretation hypotheses
echo     -binding scorers 
echo 
date
echo  ========================================================
echo
java -Djava.library.path=$CODE_HOME/output/lib -classpath $CODE_HOME/output/classes/:$CODE_HOME/tools/openccg/lib/ cast.server.CASTProcessServer -h localhost -f $CODE_HOME/subarchitectures/comsys.mk4/config/test-configurations/test6-SYS.cast
