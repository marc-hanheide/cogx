function returnBounds = showDecomposedPdf( pdf, varargin )
%
% 'linTypeSum'  ... line type of summed pdf
% 'linTypeSub'  ... line type of components
%
%
linTypeSum = 'r' ;
linTypeSub = 'k' ;
decompose = 1 ;
returnBounds = 0 ;
bounds = [] ;
showDashed = 0 ;
priorWeight = 1 ;
enumComps = 0 ;
args = varargin;
nargs = length(args);
for i=1:2:nargs
    switch args{i}
        case 'bounds', bounds = args{i+1} ;
        case 'showDashed', showDashed = args{i+1} ; 
        case 'priorWeight', priorWeight = args{i+1} ;
        case 'returnBounds', returnBounds = args{i+1} ;
        case 'decompose', decompose = args{i+1} ;
        case 'linTypeSum', linTypeSum = args{i+1} ;
        case 'linTypeSub', linTypeSub = args{i+1} ;
        case 'enumComps', enumComps = args{i+1} ;
    end
end
 
if showDashed == 1 
    linTypeSum = [linTypeSum(1),'--'] ;
elseif showDashed == 0 
    linTypeSum = [linTypeSum(1)] ; 
end

if isempty(bounds)
    b1 = sqrt(max([pdf.covariances])) ;
    bmin = min([pdf.mu]) - b1*5 ;
    bmax = max([pdf.mu]) + b1*5 ;
    bounds = [bmin,bmax] ;
end

if returnBounds > 0 & nargout > 0
    returnBounds = bounds ;
else
    returnBounds = [] ;
end

pdf.weights = priorWeight*pdf.weights ;

if decompose == 1
    h = ishold ; 
    hold on ;
    for i = 1 : length(pdf.weights)
        showPdf( bounds, 1000, pdf.mu(:,i), pdf.covariances(i,:), pdf.weights(i), linTypeSub, 2) ;
        if ( enumComps == 1 )
           text(pdf.mu(:,i), 5, num2str(i)) ;%, 'FontSize',16
        end
    end
    showPdf( bounds, 1000, pdf.mu, pdf.covariances, pdf.weights, linTypeSum,2  ) ;
    
    
    if ( h == 0 ) hold off ; end
else
    showPdf( bounds, 1000, pdf.mu, pdf.covariances, pdf.weights, linTypeSum, 2  ) ;
end



ca = axis ;
%axis([bounds,0,ca(4)]);
axis tight

% ----------------------------------------------------------------------- %
function showCurrentResult(f_ref, data, fignum)
figure(fignum); clf; 
dat_scale = 1 ;

b1 = sqrt(max([f_ref.covariances])) ;
bmin = min([f_ref.mu]) - b1*5 ;
bmax = max([f_ref.mu]) + b1*5 ;
bounds = [bmin,bmax] ;

subplot(1,3,1) ; hold on ;
[hst, hst_x] = hist(data,[-1:0.1:0]) ; hst = hst / sum(hst) ; bar(hst_x,hst) ; axis tight ; ca=axis;
line([0,0],[0,ca(4)],'color','g');
axis([[-1,1]*dat_scale,0,ca(4)]);

subplot(1,3,2) ; 
pdf_pos = constructKDEfromData( data, 'compression', 0 ) ;
for i = 1 : length(pdf_pos.weights)
   showPdf( bounds, 1000, pdf_pos.mu(:,i), pdf_pos.covariances(i,:), pdf_pos.weights(i), 'k', 1) ; 
end
showPdf( bounds, 1000, pdf_pos.mu, pdf_pos.covariances, pdf_pos.weights, 'r',2  ) ;
ca = axis ;
line([0,0],[0,ca(4)],'color','g');   
axis([[-1,1]*dat_scale,0,ca(4)]);

subplot(1,3,3) ; hold on ;
for i = 1 : length(f_ref.weights)
   showPdf( bounds, 1000, f_ref.mu(:,i), f_ref.covariances(i,:), f_ref.weights(i), 'k',1 ) ; 
end 
showPdf( bounds, 1000, f_ref.mu, f_ref.covariances, f_ref.weights, 'r',2  ) ;
ca = axis ;
line([0,0],[0,ca(4)],'color','g');   
axis([[-1,1]*dat_scale,0,ca(4)]);
 
drawnow ;
% ----------------------------------------------------------------------- %
function y_evals = showPdf( bounds, N,centers, covariances, weights, color, lw )
x_evals = [bounds(1):abs(diff(bounds))/N:bounds(2)] ;
y_evals = evaluateDistributionAt( centers, weights, covariances, x_evals ) ;
plot ( x_evals, y_evals, color, 'LineWidth',lw )

