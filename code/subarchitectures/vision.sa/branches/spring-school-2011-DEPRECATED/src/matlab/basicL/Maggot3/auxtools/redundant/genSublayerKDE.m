function pdf_out = genSublayerKDE( pdf, H_new )
% generates an equivalent pdf from the pdf's sublayer

L = [] ;
pdf_out.Mu = [] ;
pdf_out.Cov = {} ;
pdf_out.w = [] ;
for i = 1 : length(pdf.w)            
        % generate two subcomponents from the subLayer
        pdfX.w = pdf.suffStat.subLayer(i).w*pdf.w(i) ;
        pdfX.Mu = pdf.suffStat.subLayer(i).Mu ;
        pdfX.Cov = pdf.suffStat.subLayer(i).Cov ;
        
        for j = 1 : length(pdfX.w)
            pdfX.Cov{j} = H_new + pdf.suffStat.subLayer(i).A{j} -  pdfX.Mu(:,i)*pdfX.Mu(:,i)' ;
        end
        
    % augment the output kde mixture model
    pdf_out.Mu = [pdf_out.Mu, pdfX.Mu] ;
    pdf_out.Cov = horzcat( pdf_out.Cov, pdfX.Cov ) ;
    pdf_out.w = [pdf_out.w, pdfX.w] ;    
    L = [L, i*ones(1,length(pdfX.w))] ;
end
pdf_out.L = L ;