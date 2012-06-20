function compileBWcalc()
% compile mex functions for bandwidth

disp('Start compiling...')

mex -outdir ../ -v  mex_getIntSquaredHessian.cpp


disp('Done!')