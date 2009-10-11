function displayDistributionsUnlearning(pdf1, pdf2, pdf3, fig_num)
bounds = [-1.5,6] ;
figure(fig_num); clf ; hold on ;
y1 = showPdf( bounds, 100, pdf1.mu, pdf1.covariances, pdf1.weights, 'g' ) ;
a = legend ; legend([a, 'Input distribution']) ;

if ( isempty(pdf2) ) return ; end
y2 = showPdf( bounds, 100, pdf2.mu, pdf2.covariances, pdf2.weights, 'r' ) ;
a = legend ; legend([a, 'Output distribution']) ;

if ( isempty(pdf3) ) return ; end
y2 = showPdf( bounds, 100, pdf3.mu, pdf3.covariances, pdf3.weights, ':k' ) ;
a = legend ; legend([a, 'Unlearning distribution']) ;
