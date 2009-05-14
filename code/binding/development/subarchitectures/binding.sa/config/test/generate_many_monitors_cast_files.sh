#!/bin/bash

for ((a=$1; a < $2 ; a++))
do
  ./generate_many_monitors_cast_file.sh $a $3 > ./test-many-monitors_$a.cast ;
done  
./generate_many_monitors_xml_file.sh $1 $2 > test-many-monitors.xml;
