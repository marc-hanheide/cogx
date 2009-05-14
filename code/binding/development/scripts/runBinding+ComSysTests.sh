clear
echo  ===================================================
echo  Comsys.mk4 [ test harness ] 
echo  ---------------------------------------------------
echo
echo  Tests for the binding + comsys: 
echo  brief object descriptions, for testing with the playmate scenario
echo 
echo  For each test, comsys and binding subarchitectures are started up. Each test should result in a proxy structure being written to binding WM. 
echo 
echo  DOT files are being written to [ ./subarchitectures/comsys.mk4/graphs/binder ] 
echo 
echo  Author: 
echo  Pierre Lison [pierrel@coli.uni-sb.de]
echo  Geert-Jan Kruijff [ gj@dfki.de ] 
echo  Henrik Jacobsson [ henrik.jacobsson@dfki.de ] 
echo
date
echo  ===================================================
echo
echo
mkdir -p graphs

export DYLD_LIBRARY_PATH=./output/lib
export LD_PRELOAD=output/lib/libAlwaysPositiveTaskManager.so

java -classpath $CLASSPATH:./output/classes/ -Djava.library.path=./output/lib -ea cast.testing.CASTTestHarness -h localhost -f tests/Binding+ComsysTests.xml

echo
