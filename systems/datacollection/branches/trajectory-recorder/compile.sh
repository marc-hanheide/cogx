#!/bin/bash

mkdir BUILD
cd BUILD
cmake .. -C ../cmake/cache/datacollection.cmake .. 
make install

