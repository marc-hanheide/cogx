% Current directory
Current = pwd;

% CrossMod directory
addpath([Current '/CrossMod']);

% CrossMod source directory...
addpath([Current '/CrossMod/src']);

% Experiments directory...
% addpath('./experiments');

% Path to location of useful tools...
% ToolsRootPath = '../../../Tools';

% SOM Toolbox...
% http://www.cis.hut.fi/somtoolbox/
addpath([Current '/somtoolbox']);

% Earth Mover's distance MEX function...
% http://www.mathworks.com/matlabcentral/fileexchange/12936-emd-earth-mover
% s-distance-mex-interface
% http://ai.stanford.edu/~rubner/emd/default.htm
% addpath([ToolsRootPath '/Metrics/emd']);

addpath([Current '/FeatureExtraction']);