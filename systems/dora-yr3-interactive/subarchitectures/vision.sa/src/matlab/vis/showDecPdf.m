function showDecPdf(pdf,h)

if nargin<2
   h=gca;
end;

b1 = sqrt(max([cell2mat(pdf.Cov)])) ;
bmin = min([pdf.Mu]) - b1*5 ;
bmax = max([pdf.Mu]) + b1*5 ;
bounds = [bmin,bmax] ;

hold(h,'on');
for i = 1 : length(pdf.w)
   showPdf( bounds, 1000, pdf.Mu(:,i), cell2mat(pdf.Cov(i,:)), pdf.w(i), 'k', 1, h) ; 
end
showPdf( bounds, 1000, pdf.Mu, cell2mat(pdf.Cov), pdf.w, 'r',2, h  ) ;
%ca = axis ;
%axis([bounds,0,ca(4)]);
%set(h,'axis','tight');
axis(h,'tight');



% ----------------------------------------------------------------------- %

function y_evals = showPdf( bounds, N,centers, covariances, weights, color, lw, h )
x_evals = [bounds(1):abs(diff(bounds))/N:bounds(2)] ;
y_evals = evaluateDistributionAt( centers, weights, covariances, x_evals ) ;
plot (h, x_evals, y_evals, color, 'LineWidth',lw )

