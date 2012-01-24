function f = createInitialRBFset( f1, varargin )
%
% Matej Kristan (2007)
%
% Creates intial RBF distribution from f1. If parameter
% 'modify' is set to 1, then weights are transformed. 
% This is important to change a non-proper Gaussian mixture
% into proper one. E.g., non-proper mixture is when at least one of the
% weights is negative.
%

modify = 1 ;
args = varargin;
nargs = length(args);
for i=1:2:nargs
  switch args{i}
   case 'modify', modify = args{i+1};     
  end
end

if ( modify == 0 ) f = f1 ; return ; end ;

MU = [] ; COV = [] ;
for i = 1 : length(f1.weights) 
    if f1.weights(i) > 0
       MU = [MU, f1.mu(:,i)]  ;
       COV = [COV; f1.covariances(i,:)] ;
    else
       dm = [-1, 1]*sqrt(f1.covariances(i,:))*1.5 ;  
       mu = f1.mu(:,i) + dm ;
       MU = [MU, mu] ;
       C = ([-1; 1]*sqrt(f1.covariances(i,:))/1.5).^2 ;  
       COV = [COV; C] ;
    end            
end
W = evaluateDistributionAt( f1.mu, f1.weights, f1.covariances, MU ) ;
W(find(W<0)) = 0 ;
W = W / sum(W) ;
f.mu = MU ; 
f.covariances = COV ;
f.weights = W ;


% f.mu = f1.mu ;
% for i = 1 : length(f1.weights) 
%     X = sampleMixtureOfGaussians( f1.mu(:,i), 1, f1.covariances(i,:)*(0.2)^2, 1 ) ;
%     f.mu(:,i) = X ;
% end
% 
% 
% f.covariances = f1.covariances ;
%  f.weights =  ones(1,length(f.mu)) ;  %abs(f1.weights);
% f.weights = f.weights / sum(f.weights) ;