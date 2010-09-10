function pdf_res = incrementalSubOptCompression( pdf_ref, varargin )

num_components = length(pdf_ref.weights) ;
batch_step = 100 ;
args = varargin;
nargs = length(args);
for i=1:2:nargs
    switch args{i}
        case 'batch_step', batch_step = args{i+1} ;
    end
end

pdf_init.mu = [] ;
pdf_init.weights = [] ;
pdf_init.covariances = [] ;
for i = 0 : batch_step : num_components
    select = [i+1 : min([i+batch_step,num_components])] ;
    if isempty(select) break ; end
    pdf2 = select_subPdf( pdf_ref, select ) ; scale_w = sum(pdf_ref.weights(select)) ;
    pdf2 = compressDistribution( pdf2, varargin{:}, 'pruning','SMO', 'finely_refit', 1) ;
    pdf_init.mu = [pdf_init.mu, pdf2.mu] ;
    pdf_init.weights = [pdf_init.weights, pdf2.weights*scale_w] ;
    pdf_init.covariances = [pdf_init.covariances; pdf2.covariances] ;    
end
pdf_init.weights = pdf_init.weights/length(pdf_ref.weights) ;
pdf_res = compressDistribution( pdf_init, varargin{:}, 'f_init', pdf_init, 'gradient', 1 ) ;


function pdf2 = select_subPdf( pdf, select )

pdf2.mu = pdf.mu(:,select) ;
pdf2.covariances = pdf.covariances(select,:) ;
pdf2.weights = pdf.weights(select)/sum(pdf.weights(select)) ;

