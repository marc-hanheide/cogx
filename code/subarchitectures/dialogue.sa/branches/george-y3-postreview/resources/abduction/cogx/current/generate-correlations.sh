#!/bin/sh

cat correlations-prefix.txt
./generate-suffix.pl 0.7 0.2 0.3 < george-domain.txt
