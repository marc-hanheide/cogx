#!/bin/bash

chmod a+x cleandata.sh
chmod a+x packdata.sh
mkdir BUILD
cd BUILD
cmake ..
make install

