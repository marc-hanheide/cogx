#!/bin/bash

mkdir .release
cd .release
ccmake -DCMAKE_BUILD_TYPE:STRING=Release ..

