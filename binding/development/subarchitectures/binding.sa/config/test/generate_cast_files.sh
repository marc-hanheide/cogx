#!/bin/bash

for ((a=$1; a < $2 ; a++))
do
  ./generate_cast_file.sh $a > ./test-bindings_$a.cast ;
done  
./generate_xml_file.sh $1 $2 > test-bindings.xml;
