#!/bin/bash


echo "<?xml version=\"1.0\"?>"
echo "<tests>"
for ((a=$1; a < $2 ; a++))
do
echo ""
echo "  <test>"
echo ""
echo "    <name>test-many_monitors_$a</name>" ;
echo ""
echo "    <!-- cast file to run -->"
echo "    <file>subarchitectures/binding.sa/config/test/test-many-monitors_$a.cast</file>"
echo ""
echo "    <!-- description of the test -->"
echo "    <description>Test of subarchitectures/binding.sa/config/test/test-many-monitors_$a.cast. See source code or log for descriptions. </description>"
echo ""
echo "    <output>false</output>"
echo ""
echo "      <timeout>60000</timeout>"
echo ""
echo "  </test>"
done
echo ""
echo "</tests>"
