function installMainToolsPaths() 

newPath = '.\' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\compression' ; rmpath(newPath) ; addpath(newPath) ;
installCompressionPaths() ;

newPath = '..\incrementalPDF' ; rmpath(newPath) ; addpath(newPath) ;
installIncrementalKDEUpdatePaths() ;

newPath = '..\unlearning' ; rmpath(newPath) ; addpath(newPath) ;
installUnlearningPaths() ;

newPath = '..\bw_selection' ; rmpath(newPath) ; addpath(newPath) ;
installBw_selectionPaths() ;

newPath = '..\UHellinger' ; rmpath(newPath) ; addpath(newPath) ;
installUHellingerPaths() ;
