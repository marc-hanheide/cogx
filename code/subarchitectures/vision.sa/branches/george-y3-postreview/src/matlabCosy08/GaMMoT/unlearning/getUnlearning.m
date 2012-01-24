function e = getUnlearning(f, r, r_x_max)

r_y_max = evaluateDistributionAt( r.mu, r.weights, r.covariances, r_x_max ) ;
m = productMixture( f, r ) ; 
m.weights = -m.weights/r_y_max ;

e = f ;
e.mu = [f.mu, m.mu] ;
e.weights = [f.weights, m.weights] ;
e.weights = e.weights/sum(e.weights) ;
e.covariances = [f.covariances; m.covariances] ;



% m = productMixture( f, r ) ;
% r_max = evaluateDistributionAt( r.mu, r.weights, r.covariances, r_y_max ) ;
% m.weights = -m.weights/r_max ;
% 
% e.mu = [f.mu, m.mu] ;
% e.weights = [f.weights, m.weights] ;
% e.weights = e.weights/sum(e.weights) ;
% e.covariances = [f.covariances; m.covariances] ;
