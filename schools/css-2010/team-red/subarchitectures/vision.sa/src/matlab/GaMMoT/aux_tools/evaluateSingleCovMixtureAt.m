function pdf = evaluateSingleCovMixtureAt( mu, weights, covariance, locations )
% mu          ... mean values of mixture components
% weights     ... weights of mixture components
% covariances ... covariance s of components
% locations   ... points where distribution is to be evaluated

dim = cols(covariance) ;
num_data = cols(mu) ;
num_locations = cols(locations) ;

pdf = zeros(1,num_locations) ;
for i_data = 1 : num_data
    point = mu(:,i_data) ;    
    pdf = pdf + normpdf(locations,point,[],covariance)*weights(i_data) ;
end