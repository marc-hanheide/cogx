function alpha = optimizeWeights( f_ref, f_fit, C )

len_ref = length(f_ref.weights) ;
len_fit = cols(f_fit.mu) ;
P = zeros(len_fit,1) ;

d = rows(f_fit.mu) ;
C = zeros(len_fit, len_fit) ;
C_diag = zeros(1,len_fit) ;
for i = 1 : len_fit
    Cov1 = reshape(f_fit.covariances(i, :),d,d) ;
    Mu1 = f_fit.mu(:,i) ;
    C_diag(i) = integOfTwoGaussProd( Mu1, Cov1, Mu1, Cov1 ) ; 
    for j = i+1 : len_fit  
        Cov2 = reshape(f_fit.covariances(j, :),d,d) ;
        C(i,j) = integOfTwoGaussProd( Mu1, Cov1, f_fit.mu(:,j), Cov2 ) ;
    end
    
    p = 0 ;
    for j = 1 : len_ref
        Cov2 = reshape(f_ref.covariances(j, :),d,d) ;
        Mu2  = f_ref.mu(:,j) ; 
        p = p + f_ref.weights(j)*integOfTwoGaussProd( Mu1, Cov1, Mu2, Cov2 ) ;
    end
    P(i) = p ;
end
C = (C+C') + diag(C_diag) ;
 
%    alpha = reduceSolve(C,P,2)';

alpha = SMO(C,P')' ;
