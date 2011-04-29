function displayArrayOfpdfs(pdf_array, fignum)

Nd = 100 ;
num_pdfs = length(pdf_array) ;
for i = 1 : num_pdfs
    p = pdf_array{i} ;
    if i == 1 lb = p.mu(1); ub = lb ; end 
    d = max(p.covariances)*4 ;
    lb = min([lb,min(p.mu) - d]) ;
    ub = max([ub,max(p.mu) + d]) ;
end
x_evals = [lb:abs(ub-lb)/Nd:ub] ;

figure(fignum) ; clf; hold on ;
for i = 1 : num_pdfs
    y_evals = evaluateDistributionAt( pdf_array{i}.mu, pdf_array{i}.weights,...
                                      pdf_array{i}.covariances, x_evals ) ;
    plot ( x_evals, y_evals, 'b' ) ;
end
title('A set of distributions in pdf{\_}array') ;




 
