#!/bin/bash

cps="../../../../output/bin/cpvisuallearnerprebuilt.sh"

if [ -f $cps ]; then
   $cps
fi

cd glnxa64
zip -u ../libVisualLearnerCtf64 *.so *.h *.txt

cd ../glnx86
zip -u ../libVisualLearnerCtf32 *.so *.h *.txt
