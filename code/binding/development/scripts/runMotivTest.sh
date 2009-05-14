echo  ========================================================
echo  Comsys.mk4 interactive binding test 080725
echo  --------------------------------------------------------
echo  comsys.sa: subarch for situated dialogue processing
echo     - ASR, parsing [MOLOKO v5], visualization, 
echo	 - discourse referents
echo     - complete binding of PLFs on main binding.sa WM, motiv.sa WM
echo 
echo  binding.sa: subarch for indexical interpretations
echo  motiv.sa: subarch for intentional interpretation
echo 
date
echo  ========================================================
mkdir -p binding_test/dotfiles_motiv
java -Xmx2000m -ea -classpath ./output/classes/ cast.server.CASTProcessServer -h localhost -f ./subarchitectures/comsys.mk4/config/test-configurations/test-comsys_motiv.cast
