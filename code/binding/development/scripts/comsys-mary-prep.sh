#!/bin/bash

pushd tools/mary
echo "downloading mary lib"
wget  http://www.cs.bham.ac.uk/research/projects/cosy/packages/marylib.tar.gz
echo "unpacking mary lib"
tar xzvf marylib.tar.gz
rm marylib.tar.gz
popd
echo "Mary lib installed and ready to be used"
