function [scale shift]= getScaleOfDistribution( pdf )
%
% Matej Kristan (2007)
%
% Calculates the appropriate scale and shift of distribution. 
% Proper scale is important so that subsequent optimization 
% is well conditioned.
%

% acceptable minimal determinant of covariance matrix
min_Var =  1.0 ;
dim = rows(pdf.mu) ;

% get minimal variance
determinants = calculateDeterminants( pdf.covariances ) ;
v_min = (min(determinants))^(1/dim) ;

% get scale
scale = sqrt(min_Var / v_min) ;

% get shift
W = repmat(pdf.weights, rows(pdf.mu), 1) ;
shift = sum(W.*pdf.mu) ;

function determinants = calculateDeterminants( covariances )
num_points = rows(covariances) ; 
dim = sqrt(cols(covariances)) ;
len_dim = dim^2 ;

determinants = zeros( 1, num_points ) ;
for i_point = 1 : num_points
    Covariance = reshape(covariances(i_point,:),dim,dim ) ;
    determinants(i_point) = abs(det(Covariance)) ;
end