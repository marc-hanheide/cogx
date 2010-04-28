function f = combineDistributions( f1, f2 )

% f.mu = [f1.mu, f2.mu] ;
% f.covariances = [f1.covariances; f2.covariances] ;
% f.weights = [f1.weights, f2.weights]*0.5 ;

f.mu = [f1.mu, f2.mu] ;
f.mu= (f.mu - mean(f.mu))*1.0 + mean(f.mu) ;
f.covariances = ones(length(f.mu),1)*mean([f1.covariances; f2.covariances])/5 ;
%f.weights = ones(1,length(f.mu)) ; f.weights = f.weights / sum(f.weights) ;

f.weights = [f1.weights,f2.weights*0.3] ; f.weights = f.weights / sum(f.weights) ;
