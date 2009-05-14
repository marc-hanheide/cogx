clear
echo
echo  ========================================================
echo  COMSYS.MK4: 
echo  discriminative learning for statistical parse selection.
echo 
echo - Automatic generation of training examples from a small
echo   domain-specific \(playmate scenario\) CFG grammar.
echo - Averaged perceptron for online parameter learning. 
echo
echo Author:  Pierre Lison [pierrel@coli.uni-sb.de]
echo Version: 0.2
date
echo  ========================================================

java -cp $CLASSPATH:output/classes -Xms2000m -Xmx2000m -ea org.cognitivesystems.comsys.processing.parseselection.DiscriminativeLearner --configFile=subarchitectures/comsys.mk4/learning/learningconfig_online.txt
