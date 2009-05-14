clear
echo  ========================================================
echo  Comsys.mk4 [ REVIEW MEETING DEMO 4 ] 
echo  --------------------------------------------------------
echo  comsys.sa: subarch for situated dialogue processing
echo     - ASR, parsing [MOLOKO v4.1], visualization, 
echo	 - discourse referents, event structure interpretation 
echo     - complete [lvl 2] binding of PLFs on main binding.sa WM
echo 
echo  binding.sa: subarch for complete, stable interpretations
echo     -binding scorers 
echo
date
echo  ========================================================
echo
java -Xmx2000m -Djava.library.path=$CODE_HOME/output/lib -classpath $PELLET_ROOT/lib/pellet.jar:$PELLET_ROOT/lib/aterm-java-1.6.jar:$OWLAPI_ROOT/lib/owlapi-api.jar:$OWLAPI_ROOT/lib/owlapi-apibinding.jar:$CODE_HOME/output/classes/:$CODE_HOME/tools/openccg/lib/ cast.server.CASTProcessServer -h localhost -f $CODE_HOME/subarchitectures/comsys.mk4/config/demo4.cast
