#!/bin/bash

for ((a=$1; a < $2 ; a++))
do
  ./generate_coma_cast_file.sh $a > ./test-coma_$a.cast;
done  
./generate_coma_xml_file.sh $1 $2 > test-coma.xml;
