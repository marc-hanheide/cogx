function pdf2 = recombineComponensAndSublayers( reference_pdf, clustersID ) 

% recombine clustered and nonclustered sublayers
pdf2.Mu = [] ;
pdf2.Cov = {} ;
pdf2.w = [] ;
pdf2.suffStat.A = {} ;
pdf2.suffStat.B = {} ;
pdf2.suffStat.subLayer = [] ; 
subLayer = [] ;
for i = 1 : max(clustersID)
    idx_src_cmps = clustersID == i ;
    
    subLayer_t = combineSubLayersOf( reference_pdf, idx_src_cmps ) ;
    subLayer = horzcat(subLayer, subLayer_t) ;
        
    % extract submixture 
    sub_pdf.Mu = reference_pdf.Mu(:,idx_src_cmps) ;
    sub_pdf.Cov = {reference_pdf.Cov{idx_src_cmps}} ;
    sub_pdf.w = reference_pdf.w(idx_src_cmps) ;
    sub_pdf.suffStat.B = reference_pdf.suffStat.B(idx_src_cmps) ;
    sub_pdf.suffStat.A = reference_pdf.suffStat.A(idx_src_cmps) ;

    % approximate using a single pdf
    [new_mu, new_Cov, w_out] = momentMatchPdf(sub_pdf.Mu, sub_pdf.Cov, sub_pdf.w) ;
    
    % recalculate the sufficient statistics
    suffStat = calculateSuffStatOfMergedComps( sub_pdf ) ;
        
    pdf2.Mu = [pdf2.Mu,new_mu ] ;
    pdf2.Cov = horzcat(pdf2.Cov, new_Cov) ;
    pdf2.w = [pdf2.w, w_out] ;
    
    pdf2.suffStat.A = horzcat( pdf2.suffStat.A, suffStat.A ) ;
    pdf2.suffStat.B = horzcat( pdf2.suffStat.B, suffStat.B ) ;
end
pdf2.suffStat.subLayer = subLayer ;

% ------------------------------------------------------------------ %
function subLayer = combineSubLayersOf( pdf, idx_src_cmps ) 
% merge sublayers
children = pdf.suffStat.subLayer(idx_src_cmps) ;
child_weights = pdf.w(idx_src_cmps) ;


subLayer = mergeSublayersCompClustering( children, child_weights ) ;