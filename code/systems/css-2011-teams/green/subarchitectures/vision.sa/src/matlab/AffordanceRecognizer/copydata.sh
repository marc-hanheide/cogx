#!/bin/bash
# find ../../Data/ICRA2009Project -type d -name '*' -exec mkdir -p ./Data/{} \;
# find ../../Data/ICRA2009Project -type f -name '*.mat' -exec cp {} ./Data/{} \;

cd ../../Data/ICRA2009Project
find . -depth -type f -name '*.mat' | cpio --pass-through \
 --preserve-modification-time \
  --make-directories --verbose /home/barry/Research/Matlab/Projects/CogXYear2Review/Data
