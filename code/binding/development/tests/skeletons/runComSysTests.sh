#!/bin/bash

clear

LOGDIR=comsys_tests

echo
echo -en "(regeneration of test configuration files... "

python tests/generateComsysTests.py >> $LOGDIR/log_comsys_test.txt
OUT=$?
if [ $OUT -eq 0 ];then
	echo -en "\033[0;32m"
	echo -en "Successful"
	echo -en "\033[0;30m"
	echo -e ")"
else
	echo -en "\033[0;33m"
   	echo "ERROR!"
	echo -e "\033[0;30m"
fi
echo


echo  ===================================================
echo  Comsys.mk4 [ test harness ] 
echo  ---------------------------------------------------
echo
echo  11 distinct test classes are currently implemented for ComSys:
echo  - planning and realization for %s utterances
echo  - parsing of %s utterances
echo  - packing of %s logical forms
echo  - dialogue move recognition for %s utterances
echo  - discourse referent binding for %s small discourses
echo  - visual grounding of linguistic expressions for %s situations  
echo  - text-to-speech synthesis for %s utterances
echo  - event structure interpretation for %s utterances
echo  - construction of %s SDRS structures
echo  - creation of proxy factories for %s utterances
echo  "- 30 basic ComSys <--> binder interactions"
echo
echo
echo  As a whole, the %s tests should take about 30 minutes to complete.
echo  
echo  Author: Pierre Lison [pierrel@coli.uni-sb.de]
echo
echo  send comsys_tests.tar.gz to list after finishing the tests
echo
date
echo  ===================================================
echo
echo


mkdir -p $LOGDIR
rm -rf $LOGDIR/*
rm -f ./*.cto


echo -en "\033[0;34m"
echo "NOTE: detailed logging output written to $LOGDIR/log_comsys_tests.txt"
echo -e "\033[0;30m"

echo
date > $LOGDIR/log_comsys_test.txt
svn info >> $LOGDIR/log_comsys_test.txt
echo "* USER" >> $LOGDIR/log_comsys_test.txt
echo $USER >> $LOGDIR/log_comsys_test.txt
echo "* HOSTTYPE" >> $LOGDIR/log_comsys_test.txt
echo $HOSTTYPE >> $LOGDIR/log_comsys_test.txt
echo "* uname -a" >> $LOGDIR/log_comsys_test.txt
uname -a >> $LOGDIR/log_comsys_test.txt
echo "* java -version" >> $LOGDIR/log_comsys_test.txt
java -version 2>> $LOGDIR/log_comsys_test.txt
echo "* g++ --version" >> $LOGDIR/log_comsys_test.txt
echo "" >> $LOGDIR/log_comsys_test.txt
g++ --version >> $LOGDIR/log_comsys_test.txt



mkdir -p graphs


java -classpath $CLASSPATH:./output/classes/:lib/pellet.jar:lib/ -Djava.library.path=./output/lib -ea cast.testing.CASTTestHarness -h localhost -f tests/ComSysTests.xml | tee -a $LOGDIR/log_comsys_test.txt > /dev/stdout  
echo
echo

# cd subarchitectures/binding.sa/config/test/
# chmod 755 *.sh
# rm -f *.cast *.xml
# ./generate_comsys_cast_files.sh 0 30
# cd ../../../../


export DYLD_LIBRARY_PATH=./output/lib
export LD_PRELOAD=output/lib/libAlwaysPositiveTaskManager.so


# java -ea -classpath $CLASSPATH:output/classes/ cast.testing.CASTTestHarness -h localhost -f subarchitectures/binding.sa/config/test/test-comsys.xml | tee -a $LOGDIR/log_comsys_test.txt > /dev/stdout  
echo
echo
date >> $LOGDIR/log_comsys_test.txt
touch dummy.cto
touch dummy.log
mv *.cto $LOGDIR/
mv *.log $LOGDIR/
rm $LOGDIR/dummy.cto
rm $LOGDIR/dummy.log
tar czf comsys_tests.tar.gz $LOGDIR/* 
