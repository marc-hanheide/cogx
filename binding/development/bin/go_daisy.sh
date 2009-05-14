#!/bin/bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/cosy/svn.cosy/code/output/lib:/usr/local/cosy/matlab/bin/glnx86

java -classpath $CLASSPATH:/home/cosy/svn.cosy/code/output/classes caat.server.CAATProcessServer -h $1 -f ./instantiations/visual.learning.integration/config/vli-armdemo.caat
