function installPathsRoot(varargin)
% Installs global paths

% install paths for tools
dirNames = {'./auxtools','./auxtools/drawTools', './demo', './bandwidth',...
            './compression', './distanceMeasures/', ...
            './unlearning/', './vbwms/', './main/',...
            './featSelection'} ; 

% install local path
newPath = [pwd] ;
rmpath(newPath) ; addpath(newPath) ;

for i = 1 : length(dirNames)
    installPathsFrom( dirNames{i}, varargin ) ;
end

% -------------------------------------------------------------------- %
function  installPathsFrom( dirName, varargin )
% store the name of the current position
c_loc = pwd ;
 
installCall = 'installMe(varargin{:})' ;
cd( dirName ) ;
evalin('caller',installCall) ;
cd( c_loc ) ;
