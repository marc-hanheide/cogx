function demoData()

p1.Mu = [0, 2 , 4 , 5] ;
p1.Cov = {[0.5]^2, [0.5]^2 , 0 , 0 };
p1.w = [1.5 1 1 1];  p1.w = p1.w / sum(p1.w) ; 
H = 0.3^2 ;

px.Mu = [0, 2 , 4.5 ] ;
px.Cov = {[0.5]^2, [0.5]^2 , 0.3 };
px.w = [1.5 1 2];  px.w = px.w / sum(px.w) ;
p1 = px ;

figure(1) ; clf ;
subplot(1,3,1) ;
showDecomposedUniNormMixPdf( p1) ;hold on ;
plot([4 , 5], [ 0 0], 'bo') ; 
a = axis ; a = [-2.2, 7, 0, a(4)] ; axis(a) ;
subplot(1,3,2) ;
p2.Mu = 0 ; p2.w = 1 ; p2.Cov = {H} ;
showDecomposedUniNormMixPdf( p2 ) ;
axis( [-2.2, 2.2, 0, 1.5] ) ;
subplot(1,3,3) ;
% p3 = convolvethis( px, H) ;
p3 = convolvethis( p1, H) ;
showDecomposedUniNormMixPdf( p3 ) ;
a = axis ; a = [-2.2, 7, 0, 0.44] ; axis(a) ;%a(4)+0.01
subplot(1,3,1) ; axis(a) ;

function p = convolvethis( p, H)

for i = 1 : length(p.w)
   p.Cov{i} = p.Cov{i}+H ;    
end



function tt()

N = 1000 ;
[X, pdf]= generateValuesPdf( N, 'MaWa', 0, 'MyDi',2  ) ; %'MaWa', 2 ,'LoadData', 'MaWaX'


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