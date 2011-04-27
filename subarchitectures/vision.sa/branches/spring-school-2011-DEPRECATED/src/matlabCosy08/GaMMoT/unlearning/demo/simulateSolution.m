function e = simulateSolution( F, R, I )

% evaluate on interval
[x, f] =  evaluatePdf( I, 100, F.mu, F.covariances, F.weights ) ;
[x, r] =  evaluatePdf( I, 100, R.mu, R.covariances, R.weights ) ;

% get max of distribution
[r_max, i_r ]= max(r) ;

% make g
g = 1 - r/r_max ;

% dotproduct distributions
e_c = f.*g ;

% normalize to make it proper distribution
C = sum(e_c) ;
e = e_c / C ;

hold on ;
plot(x,f,'g') ; title('Input pdf')
plot(x,r,':k') ; title('Attenuation pdf')
plot(x,e,'r') ; title('Result')


% ----------------------------------------------------------------------- %
function [x_evals, y_evals] = evaluatePdf( bounds, N,centers, covariances, weights )
x_evals = [bounds(1):abs(diff(bounds))/N:bounds(2)] ;
y_evals = evaluateDistributionAt( centers, weights, covariances, x_evals ) ;
y_evals = y_evals / sum(y_evals) ;    