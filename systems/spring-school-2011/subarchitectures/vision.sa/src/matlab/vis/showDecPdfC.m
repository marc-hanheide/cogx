function showDecPdfC(pdf,color,h)

if nargin<2
   color='r';
end;

if nargin<3
   h=gca;
end;

b1 = sqrt(max([pdf.covariances])) ;
bmin = min([pdf.mu]) - b1*5 ;
bmax = max([pdf.mu]) + b1*5 ;
bounds = [bmin,bmax] ;

% hold(h,'on');
% for i = 1 : length(pdf.weights)
%    showPdf( bounds, 1000, pdf.mu(:,i), pdf.covariances(i,:), pdf.weights(i), 'k', 1, h) ; 
% end
showPdf( bounds, 1000, pdf.mu, pdf.covariances, pdf.weights, color,2, h  ) ;
%ca = axis ;
%axis([bounds,0,ca(4)]);
%set(h,'axis','tight');
axis(h,'tight');



% ----------------------------------------------------------------------- %

function y_evals = showPdf( bounds, N,centers, covariances, weights, color, lw, h )
x_evals = [bounds(1):abs(diff(bounds))/N:bounds(2)] ;
y_evals = evaluateDistributionAt( centers, weights, covariances, x_evals ) ;
plot (h, x_evals, y_evals, color, 'LineWidth',lw )

