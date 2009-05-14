clear
echo  ========================================================
echo  Comsys.mk4 [ layered-binding+comsys ] 
echo  --------------------------------------------------------
echo  comsys.sa: subarch for situated dialogue processing
echo     - ASR, parsing [MOLOKO v4.1], visualization, 
echo	 - discourse referents, event structure interpretation 
echo     - incremental binding of PLFs on virtualbinding.sa WM
echo     - complete binding of PLFs on main binding.sa WM
echo 
echo virtualbinding.sa: subarch for incremental interpretation
echo     - binding scorers
echo     - incremental PLF content
echo 
echo  binding.sa: subarch for complete, stable interpretations
echo     - binding scorers 
echo     - converged PLF content and event structure 
echo     - COMA concept binding [ bindingcoma.sa ] 
echo
date
echo  ========================================================
echo
java -Djava.library.path=$CODE_HOME/output/lib -classpath $PELLET_ROOT/lib/pellet.jar:$PELLET_ROOT/lib/aterm-java-1.6.jar:$OWLAPI_ROOT/lib/owlapi-api.jar:$OWLAPI_ROOT/lib/owlapi-apibinding.jar:$CODE_HOME/output/classes/:$CODE_HOME/tools/openccg/lib/ cast.server.CASTProcessServer -h localhost -f $CODE_HOME/subarchitectures/comsys.mk4/config/test-configurations/layered-binding+comsys-SYS.cast
