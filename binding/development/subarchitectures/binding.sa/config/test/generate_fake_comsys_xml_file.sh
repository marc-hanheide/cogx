#!/bin/bash

echo "<?xml version=\"1.0\"?>"
echo "<tests>"
for ((a=$1; a < $2 ; a++))
do
echo ""
echo "  <test>"
echo "    <!-- name of the test --> "
echo "    <name>test-fake-comsys_$a</name>"
echo ""
echo "    <!-- cast file to run -->"
echo "    <file>subarchitectures/binding.sa/config/test/test-fake-comsys_$a.cast</file>"
echo ""
echo "    <!-- description of the test -->"
echo "    <description>Test of subarchitectures/binding.sa/config/test/test-fake-comsys_$a.cast . See source code for descriptions. </description>"
echo ""
echo "    <timeout>180000</timeout> "
echo ""
echo "    <output>false</output>"
echo ""
echo "  </test>"
done
echo ""
echo "</tests>"
