#!/bin/bash


echo "<?xml version=\"1.0\"?>"
echo "<tests>"
for ((a=$1; a < $2 ; a++))
do
echo ""
echo "  <test>"
echo "    <!-- name of the test --> "
echo "    <name>test-distr-bindings_$a</name>"
echo ""
echo "    <!-- cast file to run -->"
echo "    <file>subarchitectures/binding.sa/config/test/test-distr-binding_$a.cast</file>"
echo ""
echo "    <!-- description of the test -->"
echo "    <description>Test of subarchitectures/binding.sa/config/test/test-distr-binding_$a.cast. See source code or log for descriptions. </description>"
echo ""
echo "    <output>false</output>"
echo ""
if [ $a == 7 ] ; then
  echo "      <timeout>120000</timeout>"
else
  echo "      <timeout>60000</timeout>"
fi
echo ""
echo "  </test>"
done
echo ""
echo "</tests>"
