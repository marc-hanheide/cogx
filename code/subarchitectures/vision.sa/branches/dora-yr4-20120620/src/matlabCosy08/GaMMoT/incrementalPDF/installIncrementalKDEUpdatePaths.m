function installIncrementalKDEUpdatePaths()
% installs paths to the required functions
newPath = '.\' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\aux_tools' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\compression' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\bw_selection' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\bw_selection\optimal_1d_bw_dll' ; rmpath(newPath) ; addpath(newPath) ;
installCompressionPaths() ;
