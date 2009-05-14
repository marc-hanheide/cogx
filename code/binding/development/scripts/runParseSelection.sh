clear
echo  ========================================================
echo  Comsys.mk4 parseselection test 080822
echo  --------------------------------------------------------
echo  comsys.sa: subarch for situated dialogue processing
echo     - Context-sensitive speech recognition
echo	 - incremental parsing with CCG grammar
echo	 - discourse referents bindings
echo	 - dialogue move recognition
echo     - event structure interpretation
echo	 - SDRT-like discourse structure
echo	 - *statistical parse selection*
echo 
date
echo  ========================================================
java -Xmx2000m -ea -classpath ./output/classes/ cast.server.CASTProcessServer -h 127.0.0.1 -f ./subarchitectures/comsys.mk4/config/parseselection.cast
