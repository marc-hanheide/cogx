function Covariance = plugEquationInBandwidthEstimation( data )
%
% calculates 1D plug the equation estimators and returns a diagonal
% bandwidth.
% data ... multidimensional data, each dimension in a separate row.
%

siz = size(data) ;
N = siz(2) ;

Cs = zeros(1,siz(1)) ;
epsil=1e-3 ;

% iterate through dimensions
for i = 1 : siz(1)
    X = data(i,:) ;
    [h]=slow_univariate_bandwidth_estimate_STEPI( N, X) ;
    
%     [h] = fast_univariate_bandwidth_estimate_STEPI( N, X, epsil) ;
    Cs(i) = h^2 ;
end
    
Covariance = diag(Cs) ;
