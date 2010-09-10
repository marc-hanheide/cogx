function dist = getMonteCarloDistance( pdf1, pdf2, N, varargin )

distanceFunction = 'L2Distance' ;
debugShow = 0 ; 
args = varargin;
nargs = length(args);
for i=1:2:nargs
    switch args{i}
        case 'distanceFunction', distanceFunction = args{i+1} ;      
        case 'debugShow', debugShow = args{i+1} ;
    end
end
 
% generate the proposal pdf
pdf = mixUniNormMixtures( pdf1, pdf2 ) ;

% sample from the proposal
X = generateSamplesFromNormUniPdf( pdf, N ) ;

if debugShow == 1 
    figure(1) ; clf ; showDecomposedUniNormMixPdf( pdf, 'samplesShow', X ) ;
end

% evaluate pdfs
x1 = evaluatePdfUniNormMixAt( pdf1, X ) ; 
x2 = evaluatePdfUniNormMixAt( pdf2, X ) ; 

% calculate the weights
W = evaluatePdfUniNormMixAt( pdf, X ) ;
W = 1./W ;% W = W/sum(W) ;

% evaluate the distance function
if isequal(distanceFunction,'L2Distance')
    D = L2Distance(x1, x2) ;
elseif ( isequal(distanceFunction,'HellingerDistance') )
    D = HellingerDistance(x1, x2) ;
end
% evaluate the integral
dist = sum(D.*W) ;

% -------------------------------------------------------------- %
function D = L2Distance(pdf1, pdf2)

D = (pdf1 - pdf2).^2 ;

% -------------------------------------------------------------- %
function D = HellingerDistance(pdf1, pdf2)

D = (sqrt(pdf1) - sqrt(pdf2)).^2 ;
