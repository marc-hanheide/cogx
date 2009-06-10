#!/bin/bash

for ((a=$1; a < $2 ; a++))
do
  ./generate_distr_coma_cast_file.sh $a > ./test-distr-coma_$a.cast;
done  
./generate_distr_coma_xml_file.sh $1 $2 > test-distr-coma.xml;
