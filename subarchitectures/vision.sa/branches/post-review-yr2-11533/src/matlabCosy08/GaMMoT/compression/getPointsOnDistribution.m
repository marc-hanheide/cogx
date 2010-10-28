function [X, numSigPoints ] = getPointsOnDistribution( f, k, alpha )
% calculates a set of all sigma points for all components and stores
% them in a set of column vectors.
gran = 1 ;
num_components = length(f.weights) ;
dim = rows( f.mu ) ;
numSigPoints = numPoints( dim, k ) ;
X = zeros( dim, numSigPoints*num_components ) ;
current = 1 ;
for i = 1 : num_components
    P = reshape(f.covariances(i,:),dim,dim) ;
    select = [current:current+numSigPoints-1] ;
    X(:,select) = getPoints( f.mu(:,i), P, k, alpha ) ;
    current = current + numSigPoints ;
end

% ----------------------------------------------------------------------- %
function d = numPoints( dim, k )
% d = 2*(dim + k) + 1;
 d = 2*(k - 1)*dim + 1 ;

% ----------------------------------------------------------------------- %
function X = getPoints( mu, P, k, alpha, gran )

n = size(P,1) ;
[u,s] = eig(P) ;
S = u*sqrt(s) ; 
%  dom = alpha*[0:1:k]/k ;  Mu = repmat(mu,1,2*(1 + k)) ;
  
%dom = alpha*[1/k:1/k:1-1/k] ; Mu = repmat(mu,1,2*(k - 1)) ;
dom = erfinv([1/k:1/k:1-1/k])*alpha ; Mu = repmat(mu,1,2*(k - 1)) ;

S = [S*dom, -S*dom] ;
 
X = S+Mu ;
X = [mu,X] ;
