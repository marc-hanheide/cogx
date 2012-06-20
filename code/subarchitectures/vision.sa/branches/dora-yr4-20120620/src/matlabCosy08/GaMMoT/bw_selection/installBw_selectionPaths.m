function installBw_selectionPaths()
% installs paths to the required functions
newPath = '.\' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '.\optimal_1d_bw_dll' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\aux_tools' ; rmpath(newPath) ; addpath(newPath) ;