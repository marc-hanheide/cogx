function f = getMixture( siz )
%
% Matej Kristan (2007)
%
% Randomly generates a Gaussian distribution of size 'siz'.
%

factor = 0.1 ;
f.siz = siz ;

f.means = rand(1,siz)*30 ; f.means(siz) = f.means(siz)+20 ;

f.weights = rand(1,siz) ;
f.weights = f.weights / sum(f.weights) ;
f.covariances = abs(rand(1,siz)+0.1)*2 ;

f.covariances = f.covariances*factor^2;
f.means = f.means*factor  ;