#!/bin/bash

mcr_root=/opt/MATLAB/MATLAB_Compiler_Runtime/v78
arch=glnx86

# MCR 
if [ ! -f ${mcr_root}/runtime/${arch}/libmwqhull.so ]; then
   echo "symlink libmwqhull"
   ln -s ${mcr_root}/bin/${arch}/libmwqhull.so ${mcr_root}/runtime/${arch}/
fi

matlab_root=/opt/MATLAB/matlabR2008a
# MATLAB
# rm libgcc_s.so.1 libstdc++.so.6
 #ln -s /lib/libgcc_s.so.1 libgcc_s.so.1
 #ln -s /usr/lib/libstdc++.so.6.0.9 libstdc++.so.6

function upgrade_file() {
   local orig="$1"
   local repl="$2"
   if [ -L $orig ]; then
      return
   fi
   if [ -f $orig ]; then
      echo "rename $orig"
      mv $orig ${orig}.orig
   fi
   echo "symlink $orig"
   ln -s $2 $1
}

# GCC 4.5
dir=${matlab_root}/sys/os/${arch}
upgrade_file $dir/libgcc_s.so.1     /lib/libgcc_s.so.1
upgrade_file $dir/libstdc++.so.6    /usr/lib/libstdc++.so.6.0.9
