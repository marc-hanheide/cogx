#!/bin/bash

mcr_root=/opt/MATLAB/MATLAB_Compiler_Runtime/v78
arch=glnx86

ln -s ${mcr_root}/bin/${arch}/libmwqhull.so ${mcr_root}/runtime/${arch}/

