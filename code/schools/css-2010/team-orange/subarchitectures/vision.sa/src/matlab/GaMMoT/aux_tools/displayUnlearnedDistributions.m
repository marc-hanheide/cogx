function displayUnlearnedDistributions(pdf1, pdf2, pdf3, fig_num)

b1 = sqrt(max([pdf1.covariances;pdf2.covariances;pdf3.covariances])) ;
bmin = min([pdf1.mu,pdf2.mu,pdf3.mu]) - b1*5 ;
bmax = max([pdf1.mu,pdf2.mu,pdf3.mu]) + b1*5 ;
bounds = [bmin,bmax] ;
figure(fig_num); clf ; hold on ;
y1 = showPdf( bounds, 100, pdf1.mu, pdf1.covariances, pdf1.weights, 'g' ) ;
legend('Input distribution') ;

if ( isempty(pdf2) ) return ; end
y2 = showPdf( bounds, 100, pdf2.mu, pdf2.covariances, pdf2.weights, 'r' ) ;
legend('Input distribution', 'Output distribution') ;

if ( isempty(pdf3) ) return ; end
y2 = showPdf( bounds, 100, pdf3.mu, pdf3.covariances, pdf3.weights, ':k' ) ;
legend('Input distribution', 'Output distribution', 'Unlearning distribution') ;

% ----------------------------------------------------------------------- %
function y_evals = showPdf( bounds, N,centers, covariances, weights, color )
x_evals = [bounds(1):abs(diff(bounds))/N:bounds(2)] ;
y_evals = evaluateDistributionAt( centers, weights, covariances, x_evals ) ;
norm = sum(y_evals) ;
sign_p = sum(y_evals) < 0 ; 
y_evals = y_evals / norm * (-1)^(sign_p) ;    

plot ( x_evals, y_evals, color )

% ----------------------------------------------------------------------- %
