function showMyPdf( pdf_c )
 
pdf_c.covariances = reshape(pdf_c.covariances,length(pdf_c.weights),1) ;
    
clf ;   hold on ;
showDecomposedPdf(pdf_c, 'enumComps' , 1) ; title('compressed') ;
