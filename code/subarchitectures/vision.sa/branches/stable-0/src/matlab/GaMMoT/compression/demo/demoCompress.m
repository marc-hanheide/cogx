function demoCompress()
% 
% Matej Kristan (2007)
%
% Demonstrates GaMiMoTo compression of mixtures of Gaussians.
%

% install path
newPath = '..\..\aux_tools' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\' ; rmpath(newPath) ; addpath(newPath) ;

newPath = '..\..\uHellinger' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\..\UEM\' ; rmpath(newPath) ; addpath(newPath) ;


f1_mix = getMixture( 100 ) ;
f2_mix = getMixture( 2 ) ;

f1_mix.mu = f1_mix.means ;
f1_mix.covariances = f1_mix.covariances' ;
f2_mix.mu = f2_mix.means ;
f2_mix.covariances = f2_mix.covariances' ;

f2_mix = combineDistributions( f1_mix, f2_mix ) ;

% select type of compression
select_of_compression = 'batch' ; % 'batch', 'partitioned' 

% scaling demo
% showme(f1_mix, f1_mix, 1) ;  axis tight ;
% [scale shift]= getScaleOfDistribution( f1_mix ) ;
% f1_mix = rescaleDistribution( f1_mix, scale, shift, 'forward' ) ;
% f1_mix = rescaleDistribution( f1_mix, scale, shift, 'backward' ) ;
% showme(f1_mix, f1_mix,2) ; axis tight ;


% Debug mode
% fx = f1_mix ;
% fx.covariances = fx.covariances*(1.5)  ; %*1.5^2 ; 
% alpha = optimizeWeights( f1_mix, fx ) ;
% id = find(alpha>0) ;
% f2_mix.weights = alpha(id)'; 
% % f.weights = f.weights(id)/sum(f.weights(id)) ;
% f2_mix.mu = f1_mix.mu(:,id) ;
% f2_mix.covariances = f1_mix.covariances(id,:) ;
% dL = double(getAnalyticDerivativeAt( f1_mix, f2_mix ) )
% dW = getCostderivativeAt( f1_mix, f2_mix ) 
% showme(f1_mix, f2_mix) ;  
% [mean(f1_mix.covariances), median(f1_mix.covariances), min(f1_mix.covariances)]

% show initial distribution
showme(f1_mix, f1_mix,1) ;  

% 
dt = load('..\data.txt') ;
mu = dt(:,1)' ;
f1_mix.mu = mu ;
f1_mix.covariances = ones(length(dt),1)* 0.7158^2 ;
f1_mix.weights = ones(1,length(dt)) ;
f1_mix.weights = f1_mix.weights/sum(f1_mix.weights) ;

tic 
% compress
f1 = compressDistribution( f1_mix, 'showIntermediate', 0 ) ; % SMO, tabu
T1 = toc 
%  tic
%  f1 = incrementalCompression( f1_mix, 'showIntermediate', 0, 'batch_step', 50 ) ;
% T2 = toc ;

% pdf_res = incrementalSubOptCompression( f1_mix, 'batch_step', 50  ) ;

% show result
% showme(f1_mix, f1,1) ; 

% tic
% f2 = selfOrganizeDistribution( f1_mix ) ;
% toc
showme(f1_mix, f1,2) ; 

% ----------------------------------------------------------------------- %
function showme(f_ref, f1_mix, fignum)
figure(fignum); clf; hold on ;

b1 = sqrt(max([f1_mix.covariances;f_ref.covariances])) ;
bmin = min([f1_mix.mu,f_ref.mu]) - b1*5 ;
bmax = max([f1_mix.mu,f_ref.mu]) + b1*5 ;
bounds = [bmin,bmax] ;

showPdf( bounds, 1000, f_ref.mu, f_ref.covariances, f_ref.weights, 'r' ) ;
showPdf( bounds, 1000, f1_mix.mu, f1_mix.covariances, f1_mix.weights, 'b--' ) ;
msg = sprintf('Reference (red): %d modes, Result (blue): %d modes',...
              length(f_ref.weights), length(f1_mix.weights)) ; title(msg)
drawnow ;

% ----------------------------------------------------------------------- %
function y_evals = showPdf( bounds, N,centers, covariances, weights, color )
x_evals = [bounds(1):abs(diff(bounds))/N:bounds(2)] ;
y_evals = evaluateDistributionAt( centers, weights, covariances, x_evals ) ;
plot ( x_evals, y_evals, color )