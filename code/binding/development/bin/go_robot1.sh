#!/bin/bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/$USER/svn.cosy/code/output/lib
echo $LD_LIBRARY_PATH
java -Xmx512m -classpath /home/$USER/svn.cosy/code/output/classes  cast.server.CASTProcessServer -h $1


