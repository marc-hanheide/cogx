#!/bin/bash

###################################################
# clean up in order to reduce possible confusion ##
###################################################

TEST_ID=$(whoami)_$(date +"%Y%m%d_%H%M%S")
TEST_PREAMB=$(whoami)_$(date +"%Y%m%d_")

# delete all test results from today, just to avoid confusion if the test needed to be restarted.
rm -rf $TEST_PREAMB*
mkdir -p $TEST_ID
rm -f ./*.cto

###################################################
# create a header that describe the system a bit ##
###################################################
echo ""
echo "testing and logging in $TEST_ID/log_binding_tests.txt"
echo "to view results as they are produced:"
echo "tail -f $TEST_ID/log_binding_tests.txt"
date > $TEST_ID/log_binding_tests.txt
echo "" >> $TEST_ID/log_binding_tests.txt
svn info >> $TEST_ID/log_binding_tests.txt
echo "" >> $TEST_ID/log_binding_tests.txt
echo "* USER" >> $TEST_ID/log_binding_tests.txt
echo $USER >> $TEST_ID/log_binding_tests.txt
echo "" >> $TEST_ID/log_binding_tests.txt
echo "* HOSTTYPE" >> $TEST_ID/log_binding_tests.txt
echo $HOSTTYPE >> $TEST_ID/log_binding_tests.txt
echo "" >> $TEST_ID/log_binding_tests.txt
echo "* uname -a" >> $TEST_ID/log_binding_tests.txt
uname -a >> $TEST_ID/log_binding_tests.txt
echo "" >> $TEST_ID/log_binding_tests.txt
echo "* java -version" >> $TEST_ID/log_binding_tests.txt
java -version 2>> $TEST_ID/log_binding_tests.txt
echo "" >> $TEST_ID/log_binding_tests.txt
echo "* g++ --version" >> $TEST_ID/log_binding_tests.txt
g++ --version >> $TEST_ID/log_binding_tests.txt
echo "" >> $TEST_ID/log_binding_tests.txt
echo "" >> $TEST_ID/log_binding_tests.txt
# a fix for a weird bug
export LD_PRELOAD=output/lib/libAlwaysPositiveTaskManager.so

###################################################
### BINDING TESTS #################################
###################################################
# some preps (generation of cast and xml-files)
mkdir -p binding_test
rm -rf binding_test/*
cd subarchitectures/binding.sa/config/test/
chmod 755 *.sh
rm -f *.cast *.xml
#./generate_cast_files.sh 0 32
#./generate_distr_binding_cast_files.sh 0 32
#./generate_fake_comsys_cast_files.sh 0 11
./generate_comsys_cast_files.sh 36 51 # of 36
#./generate_coma_cast_files.sh 0 7
#./generate_distr_coma_cast_files.sh 0 7
#./generate_many_monitors_cast_files.sh 0 4 8 # last number corresponds to the number of monitors
cd ../../../../
# echo ""
# echo "starting basic binding test"
# echo "" >> $TEST_ID/log_binding_tests.txt
# echo "starting basic binding test" >> $TEST_ID/log_binding_tests.txt
# java -ea -classpath $CLASSPATH:output/classes/ cast.testing.CASTTestHarness --noPID -h localhost -f subarchitectures/binding.sa/config/test/test-bindings.xml >> $TEST_ID/log_binding_tests.txt
# tail -1 $TEST_ID/log_binding_tests.txt
# date >> $TEST_ID/log_binding_tests.txt
# echo ""
# echo "starting multi subarch binding test"
# echo "" >> $TEST_ID/log_binding_tests.txt
# echo "starting multi subarch binding test" >> $TEST_ID/log_binding_tests.txt
# java -ea -classpath $CLASSPATH:output/classes/ cast.testing.CASTTestHarness --noPID -h localhost -f subarchitectures/binding.sa/config/test/test-distr-binding.xml >> $TEST_ID/log_binding_tests.txt
# tail -1 $TEST_ID/log_binding_tests.txt
# date >> $TEST_ID/log_binding_tests.txt
# echo ""
# echo "starting multi monitor binding test"
# echo "" >> $TEST_ID/log_binding_tests.txt
# echo "starting multi monitor binding test" >> $TEST_ID/log_binding_tests.txt
# java -ea -classpath $CLASSPATH:output/classes/ cast.testing.CASTTestHarness --noPID -h localhost -f subarchitectures/binding.sa/config/test/test-many-monitors.xml >> $TEST_ID/log_binding_tests.txt
# tail -1 $TEST_ID/log_binding_tests.txt
# date >> $TEST_ID/log_binding_tests.txt
# echo ""
# echo "starting fake comsys+binding test"
# echo "" >> $TEST_ID/log_binding_tests.txt
# echo "starting fake comsys+binding test" >> $TEST_ID/log_binding_tests.txt
# java -ea -classpath $CLASSPATH:output/classes/ cast.testing.CASTTestHarness --noPID -h localhost -f subarchitectures/binding.sa/config/test/test-fake-comsys.xml >> $TEST_ID/log_binding_tests.txt
# tail -1 $TEST_ID/log_binding_tests.txt
# date >> $TEST_ID/log_binding_tests.txt
echo ""
echo "starting REAL comsys+binding test"
echo "" >> $TEST_ID/log_binding_tests.txt
echo "starting REAL comsys+binding test" >> $TEST_ID/log_binding_tests.txt
java -ea -classpath $CLASSPATH:output/classes/ cast.testing.CASTTestHarness --noPID -h localhost -f subarchitectures/binding.sa/config/test/test-comsys.xml >> $TEST_ID/log_binding_tests.txt
tail -1 $TEST_ID/log_binding_tests.txt
date >> $TEST_ID/log_binding_tests.txt
# echo ""
# echo "starting coma+binding test"
# echo "" >> $TEST_ID/log_binding_tests.txt
# echo "starting coma+binding test" >> $TEST_ID/log_binding_tests.txt
# java -ea -classpath $CLASSPATH:output/classes/:lib/pellet-1.5.1/lib/aterm-java-1.6.jar:lib/pellet-1.5.1/lib/owlapi/owlapi-api.jar:lib/pellet-1.5.1/lib/owlapi/owlapi-apibinding.jar:lib/pellet-1.5.1/lib/pellet.jar:lib/crowl.jar:lib/jena.jar:lib/arq.jar cast.testing.CASTTestHarness --noPID -h localhost -f subarchitectures/binding.sa/config/test/test-coma.xml >> $TEST_ID/log_binding_tests.txt
# tail -1 $TEST_ID/log_binding_tests.txt
# date >> $TEST_ID/log_binding_tests.txt
# echo ""
# echo "starting coma+binding distributed test"
# echo "" >> $TEST_ID/log_binding_tests.txt
# echo "starting coma+binding distributed test" >> $TEST_ID/log_binding_tests.txt
# java -ea -classpath $CLASSPATH:output/classes/:lib/pellet-1.5.1/lib/aterm-java-1.6.jar:lib/pellet-1.5.1/lib/owlapi/owlapi-api.jar:lib/pellet-1.5.1/lib/owlapi/owlapi-apibinding.jar:lib/pellet-1.5.1/lib/pellet.jar:lib/crowl.jar:lib/jena.jar:lib/arq.jar cast.testing.CASTTestHarness --noPID -h localhost -f subarchitectures/binding.sa/config/test/test-distr-coma.xml >> $TEST_ID/log_binding_tests.txt
# tail -1 $TEST_ID/log_binding_tests.txt
# date >> $TEST_ID/log_binding_tests.txt
echo "comsys binding test results are stored in $TEST_ID/binding_test"
echo "comsys binding test results are stored in $TEST_ID/binding_test"  >> $TEST_ID/log_binding_tests.txt
mv binding_test $TEST_ID/

###################################################
## make reults ready for delivery #################
###################################################
date >> $TEST_ID/log_binding_tests.txt
touch dummy.cto
cp *.cto $TEST_ID/
rm $TEST_ID/dummy.cto dummy.cto

tar czf $TEST_ID.tar.gz $TEST_ID/*
echo ""
echo "$TEST_ID.tar.gz and $TEST_ID/log_binding_tests.txt created"
echo ""
