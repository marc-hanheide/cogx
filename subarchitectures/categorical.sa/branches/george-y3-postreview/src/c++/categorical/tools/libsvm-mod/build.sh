#!/bin/sh

# --------------------------
mkdir -p build
cd build
cmake -G "Unix Makefiles" .. && make &&  ln -s build/svm-predict ../svm-predict &&  ln -s build/svm-train ../svm-train &&  ln -s build/svm-reduce ../svm-reduce &&  ln -s build/svm-scale ../svm-scale  &&  ln -s build/svm-match ../svm-match
cd ..
# --------------------------

