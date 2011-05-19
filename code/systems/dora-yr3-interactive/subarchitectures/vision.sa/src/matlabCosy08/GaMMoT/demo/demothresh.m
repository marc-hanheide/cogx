function demothresh()
%
% Matej Kristan (2007)
%
% demo for calculating threshold for the integral over distribution

newPath = '..\aux_tools\' ; rmpath(newPath) ; addpath(newPath) ;

pdf.mu = 1 ;
pdf.covariances = [1] ;
pdf.weights = 1 ;
N = 500 ;
alpha = 0.1 ;

% get nonlinear threshold function
vals = getNonlinThresholdFun( pdf, N ) ;

% get threshold at alpha
threshold = getNonLinPdfThreshold( vals, alpha ) 