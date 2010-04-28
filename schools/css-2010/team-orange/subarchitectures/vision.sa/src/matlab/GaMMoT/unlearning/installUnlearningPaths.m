function installUnlearningPaths()
% installs paths to the required functions
newPath = '.\' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\vbwms\' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\aux_tools' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\compression' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\aux_tools' ; rmpath(newPath) ; addpath(newPath) ;
installCompressionPaths() ;