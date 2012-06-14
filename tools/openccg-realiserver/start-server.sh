#!/bin/sh
java -cp `find . -name '*.jar' | tr "\n" ':'` de.dfki.lt.tr.realiserver.OpenCCGRealisationServer $1