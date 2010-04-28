function demoData()

N = 1000 ;
[X, pdf]= generateValuesPdf( N, 'MaWa', 0, 'MyDi',2,'LoadData', 'MaWaX' ) ; %'MaWa', 2 ,'LoadData', 'MaWaX'


X = (X - min(X)) ;
X = (X / (max(X)-min(X)))*1.8-0.8 ;

pdf_eval = getOptimalBatchKDE( X, 'typeBandwidth', 'silverman' ) ; %'silverman' 'plugin'


figure(1) ; clf ;
showDecomposedUniNormMixPdf( pdf, 'samplesShow', X ) ;
 
hold on 
  figure(2) ; clf ;
showDecomposedUniNormMixPdf( pdf_eval, 'samplesShow', X, 'decompose', 0, 'linTypeSum', 'g' ) ;


% ------------------------------------------------------------------ %
function demoGenerateSampleDifference()

N = 50 ;
[X, pdf]= generateValuesPdf( N, 'MaWa', 1, 'MyDi', 0, 'LoadData', 'MaWa1', 'SaveData', 'MaWa1', 'PermuteData', 0 ) ; %'MaWa', 2

figure(1) ; clf ;
showDecomposedUniNormMixPdf( pdf, 'samplesShow', X ) ;

[X, pdf1]= generateValuesPdf( N, 'MaWa', 4, 'MyDi', 0 ) ; %'MaWa', 2

figure(2) ; clf ;
showDecomposedUniNormMixPdf( pdf1, 'samplesShow', X ) ;

N = 100 ;
dist = getMonteCarloDistance( pdf, pdf1, N, 'distanceFunction', 'L2Distance' ) 
dist = getMonteCarloDistance( pdf, pdf1, N, 'distanceFunction', 'HellingerDistance' ) 

% ------------------------------------------------------------------ %
function installDemoDataPaths() 
% install Kernel learning
newPath = 'D:\Work\Matlab\IncrementalKDE\GaMMoT\aux_tools\' ; rmpath(newPath) ; addpath(newPath) ;
% installIncrementalKDEUpdatePaths() ;
%  
% initial_path = pwd ;
% cd .. ; 
% installGaMMotPaths() ;
% cd( initial_path ) ;