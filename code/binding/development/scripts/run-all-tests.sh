#!/bin/bash

rm -f *.cto
echo "testing and logging in log_all_test.txt"
echo "to view results as they are produced:"
echo "tail -f log_all_test.txt"
date > log_all_test.txt
svn info >> log_all_test.txt
./run-tests >> log_all_test.txt
# ./vision-test-prep >> log_all_test.txt
./run-vision-tests >> log_all_test.txt
./run-binding-tests.sh >> log_all_test.txt
tar czf all_test.tar.gz log_all_test.txt *.cto binding_test.tar.gz 
echo "all_test.tar.gz and log_all_test.txt created"
echo ""
