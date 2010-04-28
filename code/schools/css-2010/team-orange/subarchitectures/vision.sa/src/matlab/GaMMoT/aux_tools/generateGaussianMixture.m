function f = generateGaussianMixture( N, sw ) 

if sw == 1 
    mu0 = [ 1 , 2, 5 ] ;
    covariances0 =( ([ 0.1, 0.1, 0.1 ]*6)'.^2 ) ;
    weights0 = [2, 1, 1] ; weights0 = weights0/sum(weights0) ;
    Co = 0.2 ;(std(mu0)*(4/(3*N))^(1/5))^2 ; %(3/N)^2 ; 
else
    mu0 = [ 2, 1 ] ;
    covariances0 = (0.5*[ 0.4 0.2 ])'.^2 ;
    weights0 = [1 0.5] ; weights0 = weights0/sum(weights0) ;   
    Co = 0.2  ; 
%     mu0 = [ 2 ] ;
%     covariances0 = [ 0.2 ]'.^2 ;
%     weights0 = [1] ; weights0 = weights0/sum(weights0) ;   
%     Co = 0.5^2 ; 
end

X = sampleMixtureOfGaussians( mu0, weights0, covariances0, N ) ;
N = cols(X) ;

f.mu = X ;
f.covariances = ones(N,rows(mu0))*Co ^2 ;
f.weights = ones(1,N)/N ;

if ( sw == 0 )
    f.weights = rand(size(weights0)) ;
    f.weights = f.weights / sum(f.weights) ;
end