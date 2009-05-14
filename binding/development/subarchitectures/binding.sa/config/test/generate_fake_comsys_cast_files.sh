#!/bin/bash

for ((a=$1; a < $2 ; a++))
do
  ./generate_fake_comsys_cast_file.sh $a > ./test-fake-comsys_$a.cast;
done  
./generate_fake_comsys_xml_file.sh $1 $2 > test-fake-comsys.xml;
