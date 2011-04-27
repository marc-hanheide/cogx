function X = sampleMixtureOfGaussians( centers, weights, covariances, N )

dim = rows(centers) ;
X = [] ;
for i = 1 : length(weights)
    num = max(1, round(weights(i)*N)) ;
    center = centers(:,i) ;
    covariance = covariances(i,:) ;
    covariance = reshape(covariance,dim,dim) ;
    x = randnorm( num, center,[], covariance ) ;
    X = [X,x] ;
end

p = randperm(cols(X)) ;
X = X(:,p) ;


