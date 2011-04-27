function [f0, weights_new]= removeUnitsRSDE( pruning, f0, f_ref )
% f0 ... target
% f1 ... reference
 
minm = 1 ;
maxm = 20 ;
p.f0 = f0 ;  p.f_ref = f_ref ;
%ks = golden(p,@quality,2*minm/(minm+maxm),1,2*maxm/(minm+maxm),1e-2) ; 
ks = golden(p,@quality,minm,1.5,maxm,1e-2) ; 

f0.covariances = f0.covariances*ks ;
weights_new = optimizeWeights( f_ref, f0 ) ;
f0 = pruneMixture( f0, weights_new ) ;
res = uHellingerJointSupport( f0, f_ref ) ;
 figure(5); title(sprintf('Self-inflation: %f, distance: %f',ks, res)); drawnow ;


function res = quality(alpha,p)             % Evaluate quality using ISE estimate
  fx = p.f0 ;
  fx.covariances = fx.covariances*alpha ;
  weights_new = optimizeWeights( p.f_ref, fx ) ;
  fx.weights = weights_new' ;
  M = (sum(weights_new > 0)) ; d = rows(fx.mu) ;
  Ne = M - 1 + M*d + M*d*(d+1)/2 ;
%   res = 0.5*Ne*log(p.f_ref.components)/log(2) + crossEntropyJointSupport( p.f0, fx ) ;
  res =  0.01*Ne +  crossEntropyJointSupport( p.f0, fx ) ; % 005
%   res = uHellingerJointSupport( p.f_ref, fx ) ;
%   res = abs(0.1 - uHellingerJointSupport( p.f_ref, fx )) ;
%    res = getL2error( p.f_ref, fx ) ;
  
  
function f = pruneMixture( f, alpha )
  
id = find(alpha>0) ;
f.weights = alpha(id)'; 
% f.weights = f.weights(id)/sum(f.weights(id)) ;
f.mu = f.mu(:,id) ;
f.covariances = f.covariances(id,:) ;
  
  
%   
% 
% scale_var = 2; 2 ; 4 ; % 1.5^2 ;
% 
% 
% if ( isempty(inflationVariance) )
% %     HellError = 0.1 ;
% %     [scale_var, hl ]= getOptimalScale( f0, HellError ) ;
% %     figure(5); title(sprintf('Self-inflation: %f at predefined Hellinger: %f, calculated: %f',scale_var, HellError, hl)); drawnow ;
% %     fx = f0 ;
% %     fx.covariances = fx.covariances*scale_var ;
% 
% 
% if hellErrorGlobal > 0
%     HellError = hellErrorGlobal ; %0.2 ; 0.1; 0.05; %0.05/sqrt(2) ;
% %     [scale_var, hl ]= getOptimalScale( f0, HellError ) ;
%     [scale_var, hl ]= getOptimalScale2( f0, f1, HellError ) ;
%     
%     hl = hl - hellErrorGlobal ;
%     H = uHellingerJointSupport( f0, f1 ) ;
%     figure(5); title(sprintf('Self-inflation: %1.3g at predefined Hellinger: %1.3g, Error in calculation: %1.3g, FinalDistance: %1.3g',scale_var, HellError, hl, H)); drawnow ; 
% else
%     scale_var = -hellErrorGlobal ;
%     figure(5); title(sprintf('Self-inflation predefined: %1.3g',scale_var )); drawnow ; 
% end
%     
%     fx = f0 ;
%     fx.covariances = f0.covariances * scale_var ;
% 
% %     fx = f0 ;
% %     scale_var = 1.7 ;
% %     
% %     fx.covariances = f0.covariances * scale_var ;
% % 
% %     [likRat, likRat2] = getAvLikRatioAtSigmaPoints( f0, fx ) ;
% %      figure(5); title(sprintf('Self-inflation: %1.3g at calculated ALR: %1.3g, %1.5g',scale_var, likRat*100, likRat2)); drawnow ;
% 
% else
% %     fx = f0 ;
% %     fx.covariances = fx.covariances + inflationVariance ;
% %     
% end
% 
% if isequal(pruning,'SMO')
% %    fx = f0 ;
% %    fx.covariances = fx.covariances*scale_var ; 
%  
% % hell = suHellinger( f0, fx ) ;
% % msg = sprintf('Hellinger: %f',hell) ; figure(5); title(msg) ; 
% 
%  
%       alpha = optimizeWeights( f1, fx ) ;   
%   %    alpha = optimizeWeights( f0, fx ) ;       
%    f0 = pruneMixture( f0, alpha ) ;
%    
% %    f1 = pruneMixture( f1, alpha ) ;
% %    f0.weights = f1.weights ; f0.weights = f0.weights / sum(f0.weights) ;
% %    f0.covariances = f0.covariances*(1 + (scale_var-1)*0.2) ; %fx.covariances(find(alpha>0)) ; 
% elseif isequal(pruning,'tabu')
%   %  showPdfNow(f0,f_ref_data_X, f_ref_data_Y) ;
%     alpha = optimizeWeights_MDL( f_ref_data_X, f_ref_data_Y, f0, f1 ) ;
%     f0 = pruneMixture( f0, alpha ) ;
% else
%     error(['Unknown pruning method: ',pruning]) ;
% end