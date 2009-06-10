#!/bin/bash

for ((a=$1; a < $2 ; a++))
do
  ./generate_comsys_cast_file.sh $a > ./test-comsys_$a.cast;
done  
./generate_comsys_xml_file.sh $1 $2 > test-comsys.xml;
