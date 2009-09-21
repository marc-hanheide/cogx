function pdf = evaluateDistributionAt1D( mu, weights, covariances, locations )
% mu          ... mean values of mixture components
% weights     ... weights of mixture components
% covariances ... covariance s of components
% locations   ... points where distribution is to be evaluated
 
pdf = zeros(1,length(locations)) ;
for i = 1 : length(weights)   
    pdf = pdf + normpdf(locations, mu(i),[], covariances(i))*weights(i) ; 
end