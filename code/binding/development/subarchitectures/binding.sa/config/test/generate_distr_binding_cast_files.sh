#!/bin/bash

for ((a=$1; a < $2 ; a++))
do
  ./generate_distr_binding_cast_file.sh $a > ./test-distr-binding_$a.cast ;
done  
./generate_distr_binding_xml_file.sh $1 $2 > test-distr-binding.xml;
