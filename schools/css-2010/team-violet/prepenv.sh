#!/bin/bash
# Author: mmarko

here=$(pwd)

cd output/python
ln -s -f -T ../../subarchitectures/planner.sa/src/python/domains domains

cd $here
chmod +x output/python/show_dot.sh

