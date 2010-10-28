function demoUnlearn()
 
%newPath = 'C:/Program Files/MATLAB704/work/IncrementalKDE/' ; rmpath(newPath) ; addpath(newPath) ;
% newPath = 'C:\Program Files\MATLAB704\work\IncrementalKDE\optimalIncrementalKDE\' ; rmpath(newPath) ; addpath(newPath) ;
% newPath = '..\' ; rmpath(newPath) ; addpath(newPath) ;
% newPath = '..\..\vbwms\' ; rmpath(newPath) ; addpath(newPath) ;
% newPath = '..\..\aux_tools' ; rmpath(newPath) ; addpath(newPath) ;
% newPath = '..\..\compression' ; rmpath(newPath) ; addpath(newPath) ;

installPaths() ;

repetitions = 30 ;
N = 20 ;
f = generateGaussianMixture( N, 1 ) ;
r = generateGaussianMixture( 1, 0 ) ;

% f = generateSingleGaussian( 1, 1 ) ;
% r = generateSingleGaussian( 0.2, 0.5 ) ;

tic
disp('Compressing input data...')
r = compressDistribution( r, 'showIntermediate', 0 ) ; 
f = compressDistribution( f, 'showIntermediate', 0 ) ; 
disp('Compressed.')

toc
% figure(1) ; clf ;
% simulateSolution( f, r, [-1.5, 6] ) ; return
%displayDistributions(f, r, 1) ;

s = f ;
for i = 1:repetitions 
    % e = gaussUnlearn( f, r ) ;
     e = unlearnAndCompress( f, r ) ;

    % displayUnlearnedDistributions(s, e, r, 1) ;
    % e = compressDistribution( e, 'showIntermediate', 0, 'modify', 1 ) ;

    displayUnlearnedDistributions(s, e, r, 2) ;
    msg = sprintf('Unlearning phase %d, mixture %d',i,length(e.weights)) ; title(msg) ;
    pause() ; f = e ;
end


% ------------------------------------------------------------------ %
% function e = compressDistribution(e) 
% % split distribution in positive and negative parts,
% % compress each part and mix them back together
% 
% e = removeZeroWeightsDistribution( e ) ;
% [e_neg, e_pos, w_pos, w_neg] = splitDistribution( e ) ;
% 
% % compress subparts
% if ~isempty(e_neg.weights)
%    e_neg = compressSubpart( e_neg ) ; 
% end
% 
% if ~isempty(e_pos.weights)
%    e_pos = compressSubpart( e_pos ) ; 
% end
%  
% e = composeDistribution( e_neg, e_pos, w_pos, w_neg ) ;
% e = regularizeDistribution( e ) ;
% e = removeZeroWeightsDistribution( e ) ;

% --------------------------------------------------------------------- %
function e = removeZeroWeightsDistribution( e )

minWeight = 0.01 * 1/length(e.weights) ;
id = find(abs(e.weights) > minWeight) ;

e.weights = e.weights(id) ;
e.weights = e.weights/sum(e.weights) ;
e.mu = e.mu(:,id) ;
e.covariances = e.covariances(id,:) ;

% --------------------------------------------------------------------- %
function e = regularizeDistribution( e )

disp('regularizing distribution...')
r = e ; r.weights = r.weights*(-1) ;

disp('Searching for global minimum...')
y_max = findGlobalMaximum(r) ;
disp('found.')

[e_neg, e_pos, w_pos, w_neg] = splitDistribution( e ) ;
y_evals = evaluateDistributionAt( r.mu, r.weights, r.covariances, y_max ) ;

if isempty(e_neg.weights) | y_evals <= 0
   return ; 
end

p_neg = evaluateDistributionAt( e_neg.mu, e_neg.weights, e_neg.covariances, y_max ) ;
p_pos = evaluateDistributionAt( e_pos.mu, e_pos.weights, e_pos.covariances, y_max ) ;

scale = w_pos*p_pos / (p_neg * w_neg);
e_neg.weights = e_neg.weights*scale ;
 
e = composeDistribution( e_neg, e_pos, w_pos, w_neg ) ;
disp('Regularization finished.')
% --------------------------------------------------------------------- %
function y_max = findGlobalMaximum(r)
scale_cov = (2)^2 ;

% compute boundary of search area 
[mu, covariance] = oneGaussApproximation ( r.mu, abs(r.weights), r.covariances ) ;
search_area.mu = mu ;
search_area.inv_S = inv(reshape(covariance*scale_cov, length(mu), length(mu))) ;
search_area.thresh = 2.5 ;
search_area.enforce_fixedBW = 1 ;

global plot_intermediate_results ;
plot_intermediate_results = 1 ;

% get local maximums
stopThresh = getStopThreshold( r.covariances, 'median_min' ) ;
w = 0.2 ;
g = r ; 
g.weights = [g.weights*w, 1-w] ;
g.mu = [g.mu, mu ] ;
g.covariances = [g.covariances; covariance*2 ] ;
[precisions, determinants] = getPrecisionsAndDets( g.covariances ) ;
[ centers, id_converged ] = ...
            vbwmsModeCandidates( g.mu, g.weights, g.covariances,...
                                 precisions, determinants, stopThresh, search_area ) ;
% find global maximum
y_evals = evaluateDistributionAt( r.mu, r.weights, r.covariances, centers ) ;
[a,i_pos] = max(y_evals) ;
y_max = centers(:,i_pos) ;

% --------------------------------------------------------------------- %
function e = composeDistribution( e_neg, e_pos, w_pos, w_neg )

e.mu = [e_pos.mu, e_neg.mu] ;
e.covariances = [e_pos.covariances; e_neg.covariances] ;
e.weights = [e_pos.weights*w_pos, -e_neg.weights*w_neg] ;
if (e.weights == 0) 
    e.weights(1) = e.weights(1) + 0.0001 ; 
end
e.weights = e.weights/sum(e.weights) ;

% ------------------------------------------------------------------ %
function [e_neg, e_pos, w_pos, w_neg] = splitDistribution( e )
% split distribution e into two distributions

id_neg = find(e.weights < 0) ;
id_pos = find(e.weights >= 0) ;
e_neg.mu = e.mu(:,id_neg) ;
e_neg.covariances = e.covariances(id_neg,:) ;
e_neg.weights = -e.weights(id_neg) ;
w_neg = sum(e_neg.weights) ;
e_neg.weights = e_neg.weights / w_neg ;

e_pos.mu = e.mu(:,id_pos) ;
e_pos.covariances = e.covariances(id_pos,:) ;
e_pos.weights = e.weights(id_pos) ;
w_pos = sum(e_pos.weights) ;
e_pos.weights = e_pos.weights / w_pos ;
 

% ------------------------------------------------------------------ %
function e = compressSubpart( e )

minMembers = 2 ;
scaleCovariances = 1/1.5^2 ;
plot_int_res = 0 ;
plot_final_res = 0 ;
[new_centers, new_weights, new_covariances] = ...
         approximateDensity2( e.mu, e.weights, e.covariances, ... 
                              plot_int_res , plot_final_res,...
                              minMembers, scaleCovariances ) ;
e.mu = new_centers ;
e.weights = new_weights ;
e.covariances = new_covariances ;

% ------------------------------------------------------------------ %
function [mu, covariance] = oneGaussApproximation ( mu_in, weights_in, covariances_in )

N = cols(mu_in) ;
dim = rows(mu_in) ;
 
weights_in = weights_in / sum(weights_in) ;
W = repmat(weights_in,dim,1) ;
mu0 = sum(mu_in.*W,2) ;
covariance = zeros(dim, dim) ;

for i = 1 : N
   d = mu_in(:,i) - mu0 ;
   C = reshape(covariances_in(i,:), dim, dim) ;
   covariance = covariance + (weights_in(i)*( d*transpose(d) + C )) ;  
end
covariance = reshape(covariance, 1, dim*dim ) ;
mu = mu0 ;

% ------------------------------------------------------------------ %
function installPaths() 
curr = cd ;
cd ..\..\ ;
installGaMMotPaths() ;
cd(curr) ;
