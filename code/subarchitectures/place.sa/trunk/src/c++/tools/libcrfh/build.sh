#!/bin/sh

# --------------------------
mkdir -p build
cd build
cmake -G "Unix Makefiles" .. && make
cd ..
# --------------------------

