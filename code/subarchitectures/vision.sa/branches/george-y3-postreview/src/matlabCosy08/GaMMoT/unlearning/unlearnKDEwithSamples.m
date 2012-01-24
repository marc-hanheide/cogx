function kernel_pdf_1 = unlearnKDEwithSamples( kernel_pdf, x_new, varargin )
 
compression = 1 ;
makeSeparateKDEforNeg = 0 ;

args = varargin;
nargs = length(args);
for i=1:2:nargs
    switch args{i}
        case 'compression', compression = args{i+1} ;  
        case 'makeSeparateKDEforNeg', makeSeparateKDEforNeg = args{i+1} ;
    end
end
 
% check if the number of input samples is sufficient
if cols(x_new) < 2
    makeSeparateKDEforNeg = 0 ;
end

if makeSeparateKDEforNeg == 0
    type_bw_selection = 2 ;
    kernel_pdf_x = kernel_pdf ;
    kernel_pdf_x.covariances = kernel_pdf.scale.sg ;
    kernel_pdf_x.mu = kernel_pdf.scale.mu ;
    kernel_pdf_x.weights = 1 ;

    covariance = getBW_pluginForUnlearnt( kernel_pdf_x, x_new, type_bw_selection ) ;
    kernel_pdf_1 = incorporateNewSample( kernel_pdf, covariance , x_new, compression ) ;
else
    initializationMethod = 'Silverman' ;   
    reportProgress = 0 ;
    scaleErrorThreshold = 1/0.7 ;
    hellErrorGlobal = 0.1/scaleErrorThreshold; 
    hellErrorGlobal = hellErrorGlobal*scaleErrorThreshold ;
    nMaxComponentsPrior = cols(x_new) ;
    pdf_neg = updateIKDE( [], x_new, ...
        'initialize', 1 ,'initializationMethod', initializationMethod,...
        'nMaxComponentsPrior', nMaxComponentsPrior,...
        'scaleErrorThreshold', scaleErrorThreshold,...
        'hellErrorGlobal', hellErrorGlobal) ;
    if (compression == 0) 
        kernel_pdf_1 = gaussUnlearn( kernel_pdf, pdf_neg ) ;
    else
        kernel_pdf_1 = unlearnAndCompress( kernel_pdf, pdf_neg ) ;
    end
end








% ------------------------------------------------------------------ %
function pdf_unl = incorporateNewSample( kernel_pdf, C0 , x_new, compress )

ons = ones(cols(x_new),1) ;
pdf_u.mu = x_new ;
pdf_u.covariances = C0*ons ;
pdf_u.weights = ones(cols(x_new),1)' ;
pdf_u.weights = pdf_u.weights / sum(pdf_u.weights) ;
pdf_u.components = cols(x_new) ;

% unlearn pdf
if ( compress == 0 )
    pdf_unl = gaussUnlearn( kernel_pdf, pdf_u ) ;
else
    pdf_unl = unlearnAndCompress( kernel_pdf, pdf_u ) ;
end
