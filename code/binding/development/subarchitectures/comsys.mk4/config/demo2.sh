clear
echo  ========================================================
echo  Comsys.mk4 [ REVIEW MEETING DEMO 2 ] 
echo  --------------------------------------------------------
echo  comsys.sa: subarch for situated dialogue processing
echo     - ASR, parsing [MOLOKO v4.1], visualization, 
echo	 - discourse referents, event structure interpretation 
echo     - incremental binding of PLFs on virtualbinding.sa WM
echo     - complete binding of PLFs on main binding.sa WM
echo 
echo  binding.sa: subarch for complete, stable interpretations
echo     - binding scorers 
echo     - converged PLF content and event structure 
echo     - COMA concept binding [ bindingcoma.sa ] 
echo 
echo  vision.sa: subarch for vision
echo     - simulated visual scene creation 
echo     - visual scene [ white mug, red ball ] 
echo     - connected to binding.sa
echo 
echo  coma.sa: subarch for categorical inference
echo
echo  spatial.sa: subarch for local visual scene spatial reasoning
echo     - topological spatial relations: proximity 
echo     - projective spatial relations: left, right
echo     - connected to binding.sa
echo
echo 
date
echo  ========================================================
java -Xmx2000m -Djava.library.path=$CODE_HOME/output/lib -classpath $PELLET_ROOT/lib/pellet.jar:$PELLET_ROOT/lib/aterm-java-1.6.jar:$OWLAPI_ROOT/lib/owlapi-api.jar:$OWLAPI_ROOT/lib/owlapi-apibinding.jar:$CODE_HOME/output/classes/:$CODE_HOME/tools/openccg/lib/ cast.server.CASTProcessServer -h localhost -f $CODE_HOME/subarchitectures/comsys.mk4/config/demo2.cast
