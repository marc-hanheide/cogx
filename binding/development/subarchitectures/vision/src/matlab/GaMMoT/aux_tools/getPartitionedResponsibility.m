function [R, pdf, norms] = getPartitionedResponsibility( mu, weights, covariances, locations )
% mu          ... mean values of mixture components
% weights     ... weights of mixture components
% covariances ... covariance s of components
% locations   ... points where distribution is to be evaluated

dim = sqrt(cols(covariances)) ;
num_data = cols(mu) ;
num_locations = cols(locations) ;
[precisions, determinants] = getPrecisionsAndDets( covariances ) ;

pi_2d = (2*pi)^dim ;
norms = 1./sqrt(pi_2d*determinants) ;
constantsA = weights.*norms ;

% responsibility matrix
R = zeros(num_data,num_locations) ;
for i_data = 1 : num_data
    point = mu(:,i_data) ;
    Precision_i = reshape(precisions(i_data,:),dim,dim) ;
    D_2 = sqdist(locations,point,Precision_i) ; 
    R(i_data,:) = constantsA(i_data).*exp(-0.5*D_2') ; 
end
pdf = sum(R,1) ; R = abs(R) ;