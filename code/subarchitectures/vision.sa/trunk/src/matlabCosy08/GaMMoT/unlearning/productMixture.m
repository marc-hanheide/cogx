function m = productMixture( f, r )

d = cols(f.mu(1)) ;
len = d^2 ;

n_compf = length(f.weights) ;
n_compr = length(r.weights) ;

m.mu = [] ;
m.weights = [] ;
m.covariances = [] ;
for f_i = 1 : n_compf
    for r_j = 1 : n_compr
        [mu0, P0, Z] = compProdGauss( f.mu(:,f_i), f.covariances(f_i,:), ...
                                      r.mu(:,r_j), r.covariances(r_j,:) ) ;
        m.mu = [m.mu, mu0] ;
        m.weights = [m.weights, f.weights(f_i)*r.weights(r_j)*Z] ;
        m.covariances = [m.covariances; reshape(P0, 1, len)] ;
    end      
end


function [mu0, P0, Z] = compProdGauss( mu1, covariance1, mu2, covariance2 )

d = rows(mu1) ;
P1 = reshape(covariance1, d, d) ;
P2 = reshape(covariance2, d, d) ;
invP1 = inv(P1) ; invP2 = inv(P2) ;
detP1 = det(P1) ; detP2 = det(P2) ;

[mu0, P0, Z] = productGaussians( mu1, P1, mu2, P2, invP1, invP2, detP1, detP2 ) ;