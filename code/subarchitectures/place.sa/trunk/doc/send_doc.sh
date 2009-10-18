#!/bin/bash

./docgen.sh
rsync -rz --delete ./cpp ./java pronobis@cvap94.nada.kth.se:/afs/nada.kth.se/home/d/u150naad/public_html/software/place.sa/doc
